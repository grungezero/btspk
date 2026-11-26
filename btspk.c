/*
 * btspk - play audio from sndio on a Bluetooth A2DP device
 *
 * Talks HCI to a USB Bluetooth controller through ugen(4), runs L2CAP
 * and AVDTP itself, encodes with libsbc and reads the audio from a
 * sndiod monitor sub-device.  The main process is pledged; a small USB
 * ioctl broker handles the ugen operations pledge cannot authorize.
 *
 *   btspk [-EJSqv] [-f firmware.sfi] [-u unit] info | reset | scan | upload
 *   btspk [-JSqv] [-F ms] [-b ms] [-d device] [-n frames] [-u unit]
 *       play bdaddr
 *
 * The Intel firmware loader follows FreeBSD's iwmbtfw(8), BSD-2-Clause,
 * Copyright Adrian Chadd and Vladimir Kondratyev.
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>

#include <dev/usb/usb.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <pthread.h>
#include <pwd.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sbc/sbc.h>
#include <sndio.h>

/* USB transport: Bluetooth HCI over USB, interface class e0/01/01 */
#define BT_USB_VENDOR_INTEL	0x8087
#define BT_USB_EP_CTRL		0x00
#define BT_USB_EP_EVENT		0x81	/* interrupt in: HCI events */
#define BT_USB_EP_ACL_IN	0x82	/* bulk in: ACL data */
#define BT_USB_EP_ACL_OUT	0x02	/* bulk out: ACL data */
#define BT_USB_EPNUM(ep)	((ep) & 0x0f)	/* ugen node suffix */
#define BT_USB_REQTYPE_CMD	0x20	/* host to device, class, device */
#define BT_USB_REQ_CMD		0x00
#define BT_USB_REQTYPE_EP	0x02	/* host to device, standard, endpoint */
#define BT_USB_REQ_CLEAR_FEATURE 0x01
#define BT_USB_FEAT_EP_HALT	0x0000

/* HCI sizes. */
#define BDADDR_LEN		6
#define HCI_LINK_KEY_LEN	16
#define HCI_NAME_LEN		248
#define HCI_CMD_HDR_LEN		3	/* opcode (2), parameter length (1) */
#define HCI_CMD_MAX		260

/* USB ioctl broker protocol.  No caller-supplied ioctl numbers are accepted. */
#define USB_HELPER_REQUEST	1
#define USB_HELPER_BULK_IOCTL	2
#define USB_HELPER_DATA_MAX	HCI_CMD_MAX
#define HCI_EVT_HDR_LEN		2	/* event code, parameter length */
#define HCI_ACL_MIN_LEN		27

/* HCI commands. */
#define HCI_OPCODE(ogf, ocf)	(((ogf) << 10) | (ocf))
#define HCI_OGF_LINK_CTL	0x01
#define HCI_OGF_HOST_CTL	0x03
#define HCI_OGF_INFO		0x04
#define HCI_OGF_STATUS		0x05
#define HCI_OGF_VENDOR		0x3f

#define HCI_CMD_INQUIRY		HCI_OPCODE(HCI_OGF_LINK_CTL, 0x0001)
#define HCI_CMD_CREATE_CONN	HCI_OPCODE(HCI_OGF_LINK_CTL, 0x0005)
#define HCI_CMD_DISCONNECT	HCI_OPCODE(HCI_OGF_LINK_CTL, 0x0006)
#define HCI_CMD_LINK_KEY_REPLY	HCI_OPCODE(HCI_OGF_LINK_CTL, 0x000b)
#define HCI_CMD_LINK_KEY_NEG	HCI_OPCODE(HCI_OGF_LINK_CTL, 0x000c)
#define HCI_CMD_DELETE_STORED_KEY	HCI_OPCODE(HCI_OGF_LINK_CTL, 0x0012)
#define HCI_CMD_AUTH_REQUESTED	HCI_OPCODE(HCI_OGF_LINK_CTL, 0x0011)
#define HCI_CMD_SET_ENCRYPTION	HCI_OPCODE(HCI_OGF_LINK_CTL, 0x0013)
#define HCI_CMD_REMOTE_NAME	HCI_OPCODE(HCI_OGF_LINK_CTL, 0x0019)
#define HCI_CMD_IO_CAP_REPLY	HCI_OPCODE(HCI_OGF_LINK_CTL, 0x002b)
#define HCI_CMD_USER_CONFIRM	HCI_OPCODE(HCI_OGF_LINK_CTL, 0x002c)
#define HCI_CMD_USER_CONFIRM_NEG	HCI_OPCODE(HCI_OGF_LINK_CTL, 0x002d)
#define HCI_CMD_PASSKEY_NEG	HCI_OPCODE(HCI_OGF_LINK_CTL, 0x002f)
#define HCI_CMD_SET_EVENT_MASK	HCI_OPCODE(HCI_OGF_HOST_CTL, 0x0001)
#define HCI_CMD_RESET		HCI_OPCODE(HCI_OGF_HOST_CTL, 0x0003)
#define HCI_CMD_WRITE_FLUSH_TO	HCI_OPCODE(HCI_OGF_HOST_CTL, 0x0028)
#define HCI_CMD_WRITE_SSP_MODE	HCI_OPCODE(HCI_OGF_HOST_CTL, 0x0056)
#define HCI_CMD_WRITE_SECURE_CONNECTIONS	HCI_OPCODE(HCI_OGF_HOST_CTL, 0x007a)
#define HCI_CMD_READ_BUF_SIZE	HCI_OPCODE(HCI_OGF_INFO, 0x0005)
#define HCI_CMD_READ_ENCRYPTION_KEY_SIZE	HCI_OPCODE(HCI_OGF_STATUS, 0x0008)

/* HCI events. */
#define HCI_EVT_INQUIRY_DONE	0x01
#define HCI_EVT_INQUIRY_RESULT	0x02
#define HCI_EVT_CONN_COMPLETE	0x03
#define HCI_EVT_DISCONN		0x05
#define HCI_EVT_AUTH_COMPLETE	0x06
#define HCI_EVT_REMOTE_NAME	0x07
#define HCI_EVT_ENCRYPT_CHANGE	0x08
#define HCI_EVT_CMD_COMPLETE	0x0e
#define HCI_EVT_CMD_STATUS	0x0f
#define HCI_EVT_NUM_COMPLETED	0x13
#define HCI_EVT_LINK_KEY_REQ	0x17
#define HCI_EVT_LINK_KEY_NOTIFY	0x18
#define HCI_EVT_IO_CAP_REQ	0x31
#define HCI_EVT_IO_CAP_RSP	0x32
#define HCI_EVT_USER_CONFIRM	0x33
#define HCI_EVT_USER_PASSKEY_REQ	0x34
#define HCI_EVT_USER_PASSKEY_NOTIFY	0x3b
#define HCI_EVT_SSP_COMPLETE	0x36
#define HCI_EVT_VENDOR		0xff

/* bit n of the event mask enables event n + 1 */
#define HCI_EVT_BIT(evt)	(1ULL << ((evt) - 1))
#define HCI_EVT_MASK_DEFAULT	0x00001fffffffffffULL
#define HCI_EVT_MASK_LEN	8

/* HCI field values. */
#define HCI_STATUS_OK		0x00
#define HCI_STATUS_KEY_MISSING	0x06
#define HCI_REASON_USER_TERM	0x13	/* remote user terminated */
#define HCI_LAP_GIAC		{ 0x33, 0x8b, 0x9e }
#define HCI_LAP_LEN		3
#define HCI_INQUIRY_LEN		8	/* units of 1.28 s */
#define HCI_INQUIRY_UNLIMITED	0	/* number of responses */
#define HCI_PSRM_R1		0x01	/* page scan repetition mode */
#define HCI_NO_ROLE_SWITCH	0x00	/* stay master */
#define HCI_ENCRYPT_ON		0x01
#define HCI_IO_DISPLAY_YES_NO	0x01
#define HCI_IO_NO_INPUT_OUTPUT	0x03
#define HCI_IO_KEYBOARD_DISPLAY	0x04
#define HCI_OOB_NONE		0x00
#define HCI_AUTH_BONDING_NOMITM	0x04	/* general bonding, no MITM */
#define HCI_AUTH_BONDING_MITM	0x05	/* general bonding, MITM required */
#define HCI_KEY_UNAUTH_COMBINATION	0x04
#define HCI_KEY_AUTH_COMBINATION	0x05
#define HCI_KEY_UNAUTH_COMBINATION_P256	0x07
#define HCI_KEY_AUTH_COMBINATION_P256	0x08
#define HCI_COD_MAJOR_AUDIO	0x04
#define HCI_COD_MAJOR(cod)	(((cod) >> 8) & 0x1f)
#define HCI_PKT_TYPES_ACL	0xcc18	/* DM1 DH1 DM3 DH3 DM5 DH5 */
#define HCI_SLOTS_PER_MS(ms)	((ms) * 16 / 10)	/* 0.625 ms slots */
#define HCI_SSP_ENABLE		0x01

/* event layouts */
#define HCI_INQ_ENTRY_LEN	14
#define HCI_INQ_ADDR_PSRM_RSV	9	/* addr (6), psrm (1), reserved (2) */
#define HCI_COD_LEN		3
#define HCI_CONN_COMPLETE_LEN	11
#define HCI_CONN_ADDR_OFF	3
#define HCI_DISCONN_LEN		4

/* ACL data packets. */
#define ACL_HDR_LEN		4
#define ACL_HANDLE_MASK		0x0fff
#define ACL_PB_SHIFT		12
#define ACL_PB_MASK		0x03
#define ACL_PB_START_NOFLUSH	0x00
#define ACL_PB_CONT		0x01
#define ACL_PB_START_FLUSH	0x02	/* start, automatically flushable */
#define ACL_DATA_MAX		1100

/* L2CAP. */
#define L2CAP_HDR_LEN		4	/* length (2), channel id (2) */
#define L2CAP_CMD_HDR_LEN	4	/* code, ident, length (2) */
#define L2CAP_CID_SIGNALLING	0x0001
#define L2CAP_CID_FIRST		0x0040
#define L2CAP_MTU		672
#define L2CAP_PDU_MAX		1100
#define L2CAP_SIG_MAX		128
#define L2CAP_ECHO_MAX		64

#define L2CAP_CMD_REJECT	0x01
#define L2CAP_CMD_CONN_REQ	0x02
#define L2CAP_CMD_CONN_RSP	0x03
#define L2CAP_CMD_CONFIG_REQ	0x04
#define L2CAP_CMD_CONFIG_RSP	0x05
#define L2CAP_CMD_DISCONN_REQ	0x06
#define L2CAP_CMD_DISCONN_RSP	0x07
#define L2CAP_CMD_ECHO_REQ	0x08
#define L2CAP_CMD_ECHO_RSP	0x09
#define L2CAP_CMD_INFO_REQ	0x0a
#define L2CAP_CMD_INFO_RSP	0x0b

#define L2CAP_CONN_OK		0x0000
#define L2CAP_CONN_PENDING	0x0001
#define L2CAP_CONN_BAD_PSM	0x0002
#define L2CAP_CONN_SECURITY	0x0003
#define L2CAP_CONN_NO_RESOURCES	0x0004
#define L2CAP_CONFIG_OK		0x0000
#define L2CAP_INFO_UNSUPPORTED	0x0001
#define L2CAP_REJECT_NOT_UNDERSTOOD 0x0000

#define L2CAP_OPT_MTU		0x01
#define L2CAP_OPT_MTU_LEN	2
#define L2CAP_OPT_TYPE_MASK	0x7f	/* the high bit marks a hint */
#define L2CAP_ECHO_IDENT	0xee

#define PSM_AVDTP		0x0019

/* The two AVDTP channels and their states. */
#define CH_SIGNALLING		0
#define CH_MEDIA		1
#define CH_COUNT		2

enum chan_state {
	CHAN_FAILED = -1,
	CHAN_IDLE,
	CHAN_CONNECTING,
	CHAN_OPEN
};

/* AVDTP signalling */
#define AVDTP_LABEL_MASK	0xf0
#define AVDTP_LABEL(n)		(((n) & 0x0f) << 4)
#define AVDTP_PKT_MASK		0x0c	/* single packet when zero */
#define AVDTP_MSG_MASK		0x03
#define AVDTP_MSG_COMMAND	0x00
#define AVDTP_MSG_GENERAL_REJECT 0x01
#define AVDTP_MSG_ACCEPT	0x02
#define AVDTP_HDR_LEN		2	/* label and type, signal id */

#define AVDTP_DISCOVER		0x01
#define AVDTP_GET_CAPABILITIES	0x02
#define AVDTP_SET_CONFIGURATION	0x03
#define AVDTP_OPEN		0x06
#define AVDTP_START		0x07
#define AVDTP_CLOSE		0x08

#define AVDTP_SEID_SHIFT	2
#define AVDTP_SEP_INUSE		0x02	/* in the first byte of a SEP info */
#define AVDTP_SEP_TSEP_SINK	0x08	/* in the second byte */
#define AVDTP_SEP_MEDIA_SHIFT	4
#define AVDTP_MEDIA_AUDIO	0x00
#define AVDTP_SEP_INFO_LEN	2
#define AVDTP_LOCAL_SEID	1

#define AVDTP_CAP_TRANSPORT	0x01
#define AVDTP_CAP_CODEC		0x07
#define AVDTP_CODEC_INFO_LEN	6
#define AVDTP_CODEC_SBC		0x00

/* A2DP codec information for SBC. */
#define A2DP_FREQ_44100		0x20
#define A2DP_FREQ_48000		0x10
#define A2DP_MODE_JOINT		0x01
#define A2DP_MODE_STEREO	0x02
#define A2DP_BLOCKS_16		0x10
#define A2DP_SUBBANDS_8		0x04
#define A2DP_ALLOC_LOUDNESS	0x01
#define A2DP_BITPOOL_MIN	2
#define A2DP_BITPOOL_MAX	53	/* high quality at 44.1 kHz */
#define A2DP_SAMPLES_PER_FRAME	128	/* 16 blocks times 8 subbands */

/* RTP media packets. */
#define RTP_HDR_LEN		12
#define RTP_VERSION_2		0x80
#define RTP_PT_DYNAMIC		0x60
#define SBC_PAYLOAD_HDR_LEN	1
#define MEDIA_HDR_LEN		(RTP_HDR_LEN + SBC_PAYLOAD_HDR_LEN)
#define MEDIA_MAX_FRAMES	15	/* the frame count is 4 bits */

/* Intel vendor commands and events. */
#define INTEL_CMD_RESET		HCI_OPCODE(HCI_OGF_VENDOR, 0x0001)
#define INTEL_CMD_READ_VERSION	HCI_OPCODE(HCI_OGF_VENDOR, 0x0005)
#define INTEL_CMD_SECURE_SEND	HCI_OPCODE(HCI_OGF_VENDOR, 0x0009)
#define INTEL_CMD_READ_BOOT	HCI_OPCODE(HCI_OGF_VENDOR, 0x000d)
#define INTEL_CMD_WRITE_BOOT	HCI_OPCODE(HCI_OGF_VENDOR, 0x000e)
#define INTEL_CMD_SET_EVT_MASK	HCI_OPCODE(HCI_OGF_VENDOR, 0x0052)

#define INTEL_EVT_BOOTUP	0x02	/* firmware booted */
#define INTEL_EVT_SECURE_RESULT	0x06	/* result of the download */

#define INTEL_TLV_FORMAT_ARG	0xff	/* ask Read Version for TLVs */
#define INTEL_TLV_IMAGE_TYPE	0x1c
#define INTEL_IMG_BOOTLOADER	1
#define INTEL_IMG_OPERATIONAL	3
#define INTEL_LEGACY_VER_LEN	10	/* fixed-layout version reply */
#define INTEL_FW_BOOTLOADER	0x06
#define INTEL_FW_OPERATIONAL	0x23

/* Firmware download: fragment types and the layout of an .sfi file. */
#define INTEL_FRAG_CSS		0x00	/* signed header */
#define INTEL_FRAG_PAYLOAD	0x01
#define INTEL_FRAG_SIGNATURE	0x02
#define INTEL_FRAG_PUBKEY	0x03
#define INTEL_FRAG_MAX		0xfc
#define INTEL_FRAG_ALIGN	4
#define INTEL_CSS_LEN		0x80
#define INTEL_RSA_CHUNK		0x80
#define INTEL_RSA_KEY1_OFF	0x80
#define INTEL_RSA_KEY2_OFF	0x100
#define INTEL_RSA_SIG1_OFF	0x184	/* 4 bytes of exponent skipped */
#define INTEL_RSA_SIG2_OFF	0x204
#define INTEL_RSA_HDR_LEN	644
#define INTEL_ECDSA_CHUNK	0x60
#define INTEL_ECDSA_KEY_OFF	0x80
#define INTEL_ECDSA_SIG_OFF	0xe0
#define INTEL_ECDSA_HDR_LEN	320
#define INTEL_BOOT_ACK_MAX	256

/* Boot parameters reply, status byte first. */
#define INTEL_BOOT_LEN		23
#define INTEL_BOOT_DEV_REVID	4
#define INTEL_BOOT_SECURE	6
#define INTEL_BOOT_KEY_TYPE	8
#define INTEL_BOOT_LIMITED_CCE	21

/* Reset type, patch enable, DDC reload, boot option and boot address. */
#define INTEL_RESET_BOOT	{ 0x00, 0x00, 0x00, 0x01 }
#define INTEL_RESET_BOOTLOADER	{ 0x01, 0x01, 0x01, 0x00, 0, 0, 0, 0 }
#define INTEL_RESET_LEN		8
#define INTEL_EVENT_MASK	{ 0x87, 0x0c, 0, 0, 0, 0, 0, 0 }

/* Timeouts in milliseconds. */
#define TMO_CMD			5000
#define TMO_CMD_SHORT		2000
#define TMO_INQUIRY		15000
#define TMO_NAME		5000
#define TMO_CONNECT		20000
#define TMO_AUTH		30000
#define TMO_ENCRYPT		10000
#define TMO_DISCONNECT_POLL	50
#define TMO_L2CAP_REQUEST	3000
#define TMO_L2CAP_PENDING	10000
#define TMO_L2CAP_CONFIG	5000
#define TMO_AVDTP		5000
#define TMO_USB_WRITE		1000
#define TMO_INTEL_ACK		2000
#define TMO_INTEL_DONE		5000
#define TMO_INTEL_BOOT		3000
#define TMO_SETTLE		200
#define TMO_LINK_SETUP		500
#define TMO_ECHO_PROBE		1000
#define TMO_POLL		50

/* Buffer sizes and limits. */
#define EVBUF_SIZE		1024
#define RXBUF_SIZE		4096
#define CMD_RET_MAX		256
#define MAX_INQUIRY_RESULTS	32
#define KEY_LINE_MAX		128
#define KEY_FILE_MAX		(64 * 1024)
#define INTEL_FW_MAX_SIZE	(16 * 1024 * 1024)
#define ADDR_STR_LEN		18

/* Streaming defaults; see -n, -b and -F. */
#define STREAM_FRAMES_DEFAULT	2	/* SBC frames per packet */
#define STREAM_BUFFER_MS	30	/* sndio capture buffer */
#define STREAM_BLOCK_MS		5	/* sndio capture block */
#define STREAM_NICE		-15
#define STREAM_FLUSH_MS		100	/* 0 turns the flush timeout off */

#define BTSPK_VERSION		"0.1"

struct chan {
	uint16_t	local_cid;
	uint16_t	remote_cid;
	uint16_t	out_mtu;
	enum chan_state	state;
	int		config_in;
	int		config_out;
	int		result;		/* last connection response */
};

struct inquiry_result {
	uint8_t		bdaddr[BDADDR_LEN];
	uint32_t	class_of_device;
};

struct a2dp {
	int		sample_rate;
	int		sbc_mode;
	int		sbc_bitpool;
	uint8_t		remote_seid;
	uint8_t		codec_info[4];
};

struct usb_helper_msg;

static unsigned int	get_le16(const uint8_t *);
static void	put_le16(uint8_t *, unsigned int);
static void	put_be16(uint8_t *, unsigned int);
static void	put_be32(uint8_t *, uint32_t);
static int64_t	now_ms(void);
static int	hexval(int);
static int	bd_aton(const char *, uint8_t *);
static const char *	bd_ntoa(const uint8_t *);
static void	trace(const char *, unsigned int, const uint8_t *, int);
static const char *	keypath(void);
static FILE *	key_fopen(const char *);
static int	key_load(const uint8_t *, uint8_t *);
static int	key_save(const uint8_t *, const uint8_t *, int);
static void	keypath_init(void);
static void	set_timeout(int);
static void	usb_open_endpoints(void);
static void	usb_clear_halt(int);
static int	usb_do_request(struct usb_ctl_request *);
static int	usb_bulk_ioctl(unsigned long, int *);
static void	usb_helper_start(void);
static void	usb_helper_loop(int);
static int	usb_helper_call(struct usb_helper_msg *);
static int	usb_hci_command_allowed(uint16_t, int);
static void	check_rejected_key(void);
static int	numeric_comparison_capable(int);
static int	read_full(int, void *, size_t);
static int	write_full(int, const void *, size_t);
static void	pledge_main(int);
static void	usb_open(int);
static void	usb_reopen(void);
static void *	bulk_reader(void *);
static void	bulk_start(void);
static void	hci_cmd(uint16_t, const void *, int);
static int	wait_flag(int *, int);
static int	hci_cmd_try(uint16_t, const void *, int, int);
static int	hci_cmd_wait(uint16_t, const void *, int);
static void	acl_send(const uint8_t *, int, int);
static void	l2cap_send(struct chan *, const uint8_t *, int);
static void	sig_send(uint8_t, uint8_t, const uint8_t *, int);
static struct chan *	chan_by_cid(unsigned int);
static int	config_options(struct chan *, const uint8_t *, int);
static void	sig_input(const uint8_t *, int);
static void	l2cap_input(const uint8_t *, int);
static const char *	conn_failure(struct chan *);
static int	l2cap_connect(int, unsigned int);
static void	inquiry_result_event(const uint8_t *, int);
static void	link_key_request(const uint8_t *);
static int	is_remote(const uint8_t *, int);
static int	is_acl_handle(const uint8_t *);
static void	handle_event(int, const uint8_t *, int);
static void	acl_input(const uint8_t *, int);
static void	bulk_parse(void);
static void	pump(int);
static int	av_cmd(uint8_t, const uint8_t *, int, uint8_t *);
static int	intel_version(int);
static int	intel_boot_params(void);
static void	intel_fragment(uint8_t, const uint8_t *, size_t);
static uint8_t *	firmware_read(const char *, size_t *);
static void	intel_boot(const char *, int);
static void	upload(const char *, int);
static void	intel_reset(void);
static void	intel_setup(const char *, int, int);
static void	hci_init(void);
static void	scan(void);
static void	disconnect(void);
static void	link_up(void);
static void	set_flush_timeout(void);
static void	a2dp_negotiate(struct a2dp *);
static void	stream(struct a2dp *, const char *, int, int);
static void	play(const char *, const char *, int, int);
static void	drop_privileges(void);
static int	confirm_numeric(const uint8_t *);
static void	sandbox(const char *);
static void	onsig(int);
static __dead void	usage(void);

static volatile sig_atomic_t quit;
static int verbose;
static int quiet;
static int skip_security;	/* diagnostic only: bypass authentication */
static int allow_just_works;	/* explicit opt-in; no MITM protection */
static int flush_ms = STREAM_FLUSH_MS;
static int pump_active;
static int disconnecting;


/* Suppress informational output and non-fatal warnings with -q. */
static void
qprintf(const char *fmt, ...)
{
	va_list ap;

	if (quiet)
		return;
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
}

static void
qfprintf(FILE *fp, const char *fmt, ...)
{
	va_list ap;

	if (quiet)
		return;
	va_start(ap, fmt);
	vfprintf(fp, fmt, ap);
	va_end(ap);
}

static void
qwarnx(const char *fmt, ...)
{
	va_list ap;

	if (quiet)
		return;
	va_start(ap, fmt);
	vwarnx(fmt, ap);
	va_end(ap);
}

static void
qwarn(const char *fmt, ...)
{
	va_list ap;

	if (quiet)
		return;
	va_start(ap, fmt);
	vwarn(fmt, ap);
	va_end(ap);
}

/* USB endpoints and what was read from them */
static int fd_control = -1;
static int fd_event = -1;
static int fd_bulk = -1;
static int fd_usb_helper = -1;

/* The helper receives only fixed USB requests and bounded transfer data. */
struct usb_helper_msg {
	uint32_t		op;
	unsigned long	request;
	int32_t		value;
	uint32_t		len;
	struct usb_ctl_request ur;
	uint8_t		data[USB_HELPER_DATA_MAX];
};

struct usb_helper_reply {
	int32_t		result;
	int32_t		error;
};
static int ugen_unit;
static uint8_t event_buf[EVBUF_SIZE];
static uint8_t bulk_buf[RXBUF_SIZE];
static int event_len;
static int bulk_len;
static int bulk_read_ahead;
static int bulk_raw_mode;	/* the Intel loader reads bulk-in itself */
static pthread_mutex_t bulk_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t bulk_thread;
static int bulk_thread_started;
static int wake_pipe[2] = { -1, -1 };

/* controller and link */
static int acl_mtu = HCI_ACL_MIN_LEN;
static int acl_credits = 1;
static int acl_credits_max = 1;
static uint16_t acl_handle;
static int connected;
static uint8_t remote_bdaddr[BDADDR_LEN];
static int remote_io_capability = -1;

/* what the event handlers hand to the code that waits for them */
static int waiting_opcode = -1;
static int command_status;
static uint8_t command_reply[CMD_RET_MAX];
static int command_reply_len;
static int command_done;
static int connection_done;
static int authentication_done;
static int encryption_done;
static int rejected_link_key;
static int key_delete_done;
static int key_delete_status;
static int inquiry_done;
static int remote_name_done;
static int echo_reply_ready;
static int av_reply_ready;
static int conn_status;
static int auth_status;
static int enc_status;
static char remote_name[HCI_NAME_LEN + 1];

static int vendor_secure_done;
static int vendor_secure_status;
static int vendor_boot_done;

static struct inquiry_result inquiry_results[MAX_INQUIRY_RESULTS];
static int inquiry_count;

static struct chan channels[CH_COUNT];
static uint8_t av_rsp[256];
static int av_rsp_len;
static const uint8_t echo_payload[] = { 'b', 't', 's', 'p' };

/* Byte order and time. */

static unsigned int
get_le16(const uint8_t *p)
{
	return p[0] | p[1] << 8;
}

static void
put_le16(uint8_t *p, unsigned int v)
{
	p[0] = v;
	p[1] = v >> 8;
}

static void
put_be16(uint8_t *p, unsigned int v)
{
	p[0] = v >> 8;
	p[1] = v;
}

static void
put_be32(uint8_t *p, uint32_t v)
{
	put_be16(p, v >> 16);
	put_be16(p + 2, v & 0xffff);
}

static int64_t
now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* Bluetooth addresses: text is aa:bb:cc:dd:ee:ff, HCI wants it reversed */

static int
hexval(int c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

static int
bd_aton(const char *s, uint8_t *addr)
{
	int hi, lo, i;

	if (strlen(s) != ADDR_STR_LEN - 1)
		return -1;
	for (i = 0; i < BDADDR_LEN; i++) {
		if (i > 0 && s[i * 3 - 1] != ':')
			return -1;
		hi = hexval((unsigned char)s[i * 3]);
		lo = hexval((unsigned char)s[i * 3 + 1]);
		if (hi == -1 || lo == -1)
			return -1;
		addr[BDADDR_LEN - 1 - i] = hi << 4 | lo;
	}
	return 0;
}

static const char *
bd_ntoa(const uint8_t *a)
{
	static char buf[ADDR_STR_LEN];

	snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
	    a[5], a[4], a[3], a[2], a[1], a[0]);
	return buf;
}

/* hex dump to stderr for -v; callers cut link keys out of what they pass */
static void
trace(const char *tag, unsigned int code, const uint8_t *p, int n)
{
	int i;

	if (!verbose || (strcmp(tag, "cmd") != 0 &&
	    strcmp(tag, "evt") != 0))
		return;
	if (code == HCI_EVT_USER_CONFIRM ||
	    code == HCI_EVT_USER_PASSKEY_NOTIFY)
		n = n > BDADDR_LEN ? BDADDR_LEN : n;
	qfprintf(stderr, "%s %04x:", tag, code);
	for (i = 0; i < n; i++)
		qfprintf(stderr, " %02x", p[i]);
	fputc('\n', stderr);
}

/*
 * Link keys live in ~/.btspk/keys as address, key and HCI key type.
 * Only authenticated combination keys are loaded; the file must belong
 * to us and be private, or it is not used.
 */

static char key_dir[PATH_MAX];
static char key_file[PATH_MAX];

/* Resolve the key path and create its private directory before unveil(). */
static void
keypath_init(void)
{
	struct passwd *pw;
	struct stat st;

	if ((pw = getpwuid(getuid())) == NULL || pw->pw_dir == NULL)
		errx(1, "cannot determine the user's home directory");
	if (snprintf(key_dir, sizeof(key_dir), "%s/.btspk",
	    pw->pw_dir) >= (int)sizeof(key_dir) ||
	    snprintf(key_file, sizeof(key_file), "%s/keys",
	    key_dir) >= (int)sizeof(key_file))
		errx(1, "btspk key path is too long");

	if (mkdir(key_dir, 0700) == -1 && errno != EEXIST)
		err(1, "mkdir %s", key_dir);
	if (stat(key_dir, &st) == -1)
		err(1, "stat %s", key_dir);
	if (!S_ISDIR(st.st_mode) || st.st_uid != geteuid() ||
	    (st.st_mode & (S_IRWXG | S_IRWXO)))
		errx(1, "%s: directory must be private to the user", key_dir);
}

static const char *
keypath(void)
{
	return key_file[0] != '\0' ? key_file : NULL;
}

/* errno is ENOENT when there is simply no key file yet. */
static FILE *
key_fopen(const char *path)
{
	struct stat st;
	FILE *f;
	int fd;

	if ((fd = open(path, O_RDONLY | O_NOFOLLOW | O_NONBLOCK)) == -1)
		return NULL;
	if (fstat(fd, &st) == -1 || !S_ISREG(st.st_mode) ||
	    st.st_uid != geteuid() || (st.st_mode & (S_IRWXG | S_IRWXO)) ||
	    st.st_size > KEY_FILE_MAX) {
		close(fd);
		qwarnx("%s: ignoring key file, it must be a private file "
		    "of yours", path);
		errno = EACCES;
		return NULL;
	}
	if ((f = fdopen(fd, "r")) == NULL)
		close(fd);
	return f;
}

static int
key_load(const uint8_t *addr, uint8_t *key)
{
	char line[KEY_LINE_MAX], a[ADDR_STR_LEN];
	char k[2 * HCI_LINK_KEY_LEN + 1];
	uint8_t t[BDADDR_LEN];
	const char *path;
	FILE *f;
	int i, hi, lo, type, ok;

	ok = 0;

	if ((path = keypath()) == NULL || (f = key_fopen(path)) == NULL)
		return 0;
	while (!ok && fgets(line, sizeof(line), f) != NULL) {
		if (sscanf(line, "%17s %32s %2x", a, k, &type) != 3 ||
		    strlen(k) != sizeof(k) - 1 ||
		    (type != HCI_KEY_AUTH_COMBINATION &&
		    type != HCI_KEY_AUTH_COMBINATION_P256 &&
		    (!allow_just_works ||
		    (type != HCI_KEY_UNAUTH_COMBINATION &&
		    type != HCI_KEY_UNAUTH_COMBINATION_P256))) ||
		    bd_aton(a, t) == -1 || memcmp(t, addr, BDADDR_LEN) != 0)
			continue;
		for (i = 0; i < HCI_LINK_KEY_LEN; i++) {
			hi = hexval((unsigned char)k[2 * i]);
			lo = hexval((unsigned char)k[2 * i + 1]);
			if (hi == -1 || lo == -1)
				break;
			key[i] = hi << 4 | lo;
		}
		ok = i == HCI_LINK_KEY_LEN;
	}
	fclose(f);
	explicit_bzero(line, sizeof(line));
	explicit_bzero(k, sizeof(k));
	return ok;
}

/* Replace the entry for addr; with no key the entry is just removed. */
static int
key_save(const uint8_t *addr, const uint8_t *key, int type)
{
	char tmp[PATH_MAX + 16], line[KEY_LINE_MAX], a[ADDR_STR_LEN];
	uint8_t t[BDADDR_LEN];
	const char *path;
	FILE *in, *out;
	int fd, i, bad;

	bad = 0;

	if ((path = keypath()) == NULL) {
		qwarnx("cannot determine home directory; link key not saved");
		return 0;
	}
	in = key_fopen(path);
	if (in == NULL && errno != ENOENT)
		return 0;
	snprintf(tmp, sizeof(tmp), "%s.XXXXXXXXXX", path);
	if ((fd = mkstemp(tmp)) == -1) {
		qwarn("%s", tmp);
		if (in != NULL)
			fclose(in);
		return 0;
	}
	if ((out = fdopen(fd, "w")) == NULL) {
		qwarn("%s", tmp);
		close(fd);
		unlink(tmp);
		if (in != NULL)
			fclose(in);
		return 0;
	}
	while (in != NULL && fgets(line, sizeof(line), in) != NULL) {
		if (sscanf(line, "%17s", a) == 1 && bd_aton(a, t) == 0 &&
		    memcmp(t, addr, BDADDR_LEN) == 0)
			continue;
		if (fputs(line, out) == EOF)
			bad = 1;
	}
	if (in != NULL)
		fclose(in);
	if (key != NULL) {
		fprintf(out, "%s ", bd_ntoa(addr));
		for (i = 0; i < HCI_LINK_KEY_LEN; i++)
			fprintf(out, "%02x", key[i]);
		fprintf(out, " %02x\n", type);
	}
	if (fflush(out) == EOF || ferror(out) || fsync(fd) == -1)
		bad = 1;
	if (fclose(out) == EOF)
		bad = 1;
	if (bad || rename(tmp, path) == -1) {
		qwarn("could not write %s", path);
		unlink(tmp);
		explicit_bzero(line, sizeof(line));
		return 0;
	}
	explicit_bzero(line, sizeof(line));
	return 1;
}

/*
 * USB: control endpoint for commands, interrupt endpoint for events,
 * bulk endpoints for ACL data.  ugen allows one open per endpoint
 * number, so the bulk pair shares one descriptor.
 */

/* Bulk reads need a timeout unless the receive thread owns them. */

static void
set_timeout(int ms)
{
	if (!bulk_read_ahead && !bulk_thread_started)
		usb_bulk_ioctl(USB_SET_TIMEOUT, &ms);
}

static void
usb_open_endpoints(void)
{
	char path[32];
	int one, ms;

	one = 1;
	ms = TMO_USB_WRITE;

	snprintf(path, sizeof(path), "/dev/ugen%d.%02d", ugen_unit,
	    BT_USB_EPNUM(BT_USB_EP_EVENT));
	if ((fd_event = open(path, O_RDONLY | O_NONBLOCK)) == -1)
		err(1, "%s", path);
	snprintf(path, sizeof(path), "/dev/ugen%d.%02d", ugen_unit,
	    BT_USB_EPNUM(BT_USB_EP_ACL_OUT));
	if ((fd_bulk = open(path, O_RDWR)) == -1)
		err(1, "%s", path);

	usb_bulk_ioctl(USB_SET_SHORT_XFER, &one);
#ifdef USB_SET_BULK_RA
	if (usb_bulk_ioctl(USB_SET_BULK_RA, &one) == 0)
		bulk_read_ahead = 1;
#endif
	usb_bulk_ioctl(USB_SET_TIMEOUT, &ms);

	/*
	 * The device keeps its data toggles from the last run, ours start
	 * at DATA0, and on a mismatch the first packet vanishes.  iwmbtfw
	 * resets the whole device; ugen can only clear each endpoint.
	 */
	usb_clear_halt(BT_USB_EP_EVENT);
	usb_clear_halt(BT_USB_EP_ACL_IN);
	usb_clear_halt(BT_USB_EP_ACL_OUT);
}

static void
usb_clear_halt(int ep)
{
	struct usb_ctl_request ur;

	memset(&ur, 0, sizeof(ur));
	ur.ucr_request.bmRequestType = BT_USB_REQTYPE_EP;
	ur.ucr_request.bRequest = BT_USB_REQ_CLEAR_FEATURE;
	USETW(ur.ucr_request.wValue, BT_USB_FEAT_EP_HALT);
	USETW(ur.ucr_request.wIndex, ep);
	USETW(ur.ucr_request.wLength, 0);
	if (usb_do_request(&ur) == -1 && verbose)
		qwarn("clear halt on endpoint %02x", ep);
}

static void
usb_open(int unit)
{
	char path[32];

	ugen_unit = unit;
	snprintf(path, sizeof(path), "/dev/ugen%d.%02d", unit,
	    BT_USB_EPNUM(BT_USB_EP_CTRL));
	if ((fd_control = open(path, O_RDWR)) == -1)
		err(1, "%s", path);
	usb_open_endpoints();
}

/* The controller restarted; start over on fresh endpoints. */
static void
usb_reopen(void)
{
	close(fd_event);
	close(fd_bulk);
	event_len = bulk_len = 0;
	usb_open_endpoints();
}

/*
 * Keep one blocking read pending on bulk-in, like ng_ubt does on
 * FreeBSD.  Reading with short timeouts makes ugen abort the transfer
 * and clear the halt every time, which loses packets.
 */
static void *
bulk_reader(void *arg)
{
	uint8_t tmp[1024];
	sigset_t all;
	ssize_t n;

	sigfillset(&all);
	pthread_sigmask(SIG_BLOCK, &all, NULL);
	for (;;) {
		if ((n = read(fd_bulk, tmp, sizeof(tmp))) <= 0) {
			usleep(10000);
			continue;
		}
		trace("bulk<", 0, tmp, n < 64 ? n : 64);
		pthread_mutex_lock(&bulk_mutex);
		if (bulk_len + n > (int)sizeof(bulk_buf))
			bulk_len = 0;
		memcpy(bulk_buf + bulk_len, tmp, n);
		bulk_len += n;
		pthread_mutex_unlock(&bulk_mutex);
		explicit_bzero(tmp, sizeof(tmp));
		(void)!write(wake_pipe[1], "x", 1);
	}
	return NULL;
}

static void
bulk_start(void)
{
	int i, zero;

	zero = 0;

	if (bulk_read_ahead || bulk_thread_started)
		return;
	if (pipe(wake_pipe) == -1)
		err(1, "pipe");
	for (i = 0; i < 2; i++)
		if (fcntl(wake_pipe[i], F_SETFL, O_NONBLOCK) == -1)
			err(1, "fcntl");
	usb_bulk_ioctl(USB_SET_TIMEOUT, &zero);	/* never time out */
	bulk_thread_started = 1;
	if ((i = pthread_create(&bulk_thread, NULL, bulk_reader, NULL)))
		errx(1, "pthread_create: %s", strerror(i));
}

/* HCI commands. */

static void
hci_cmd(uint16_t op, const void *p, int plen)
{
	struct usb_ctl_request ur;
	uint8_t buf[HCI_CMD_MAX];

	if (plen < 0 || plen > 255 ||
	    (plen > 0 && p == NULL))
		errx(1, "invalid HCI command length %d", plen);
	put_le16(buf, op);
	buf[2] = plen;
	if (plen > 0)
		memcpy(buf + HCI_CMD_HDR_LEN, p, plen);
	trace("cmd", op, p, op == HCI_CMD_LINK_KEY_REPLY ? BDADDR_LEN : plen);

	if (op == INTEL_CMD_SECURE_SEND) {
		/* the Intel bootloader wants this one on bulk-out */
		set_timeout(TMO_USB_WRITE);
		if (write(fd_bulk, buf, HCI_CMD_HDR_LEN + plen) !=
		    HCI_CMD_HDR_LEN + plen)
			err(1, "bulk write of hci command %04x", op);
		explicit_bzero(buf, sizeof(buf));
		return;
	}
	memset(&ur, 0, sizeof(ur));
	ur.ucr_request.bmRequestType = BT_USB_REQTYPE_CMD;
	ur.ucr_request.bRequest = BT_USB_REQ_CMD;
	USETW(ur.ucr_request.wValue, 0);
	USETW(ur.ucr_request.wIndex, 0);
	USETW(ur.ucr_request.wLength, HCI_CMD_HDR_LEN + plen);
	ur.ucr_data = buf;
	if (usb_do_request(&ur) == -1)
		err(1, "hci command %04x", op);
	explicit_bzero(buf, sizeof(buf));
}

static int
wait_flag(int *flag, int ms)
{
	int64_t end;

	end = now_ms() + ms;

	while (!*flag && !quit && now_ms() < end)
		pump(TMO_POLL);
	return *flag;
}

/* send a command, wait for its reply; -1 on timeout, else the status */
static int
hci_cmd_try(uint16_t op, const void *p, int plen, int ms)
{
	int done;

	waiting_opcode = op;
	command_done = 0;
	hci_cmd(op, p, plen);
	done = wait_flag(&command_done, ms);
	waiting_opcode = -1;
	return done ? command_status : -1;
}

static int
hci_cmd_wait(uint16_t op, const void *p, int plen)
{
	int status;

	if ((status = hci_cmd_try(op, p, plen, TMO_CMD)) == -1)
		errx(1, "hci command %04x timed out", op);
	return status;
}

/* Cut an L2CAP PDU into ACL packets and send them, honoring credits. */

static void
acl_send(const uint8_t *pdu, int len, int flushable)
{
	uint8_t buf[ACL_HDR_LEN + ACL_DATA_MAX];
	int off, n, pb;

	off = 0;

	pb = flushable ? ACL_PB_START_FLUSH : ACL_PB_START_NOFLUSH;
	while (off < len) {
		n = len - off;
		if (n > acl_mtu)
			n = acl_mtu;
		while (acl_credits < 1) {
			if (pump_active || !connected || quit)
				return;
			pump(20);
		}
		if (!connected)
			return;
		put_le16(buf, (acl_handle & ACL_HANDLE_MASK) |
		    pb << ACL_PB_SHIFT);
		put_le16(buf + 2, n);
		memcpy(buf + ACL_HDR_LEN, pdu + off, n);
		trace("acl>", 0, buf, n + ACL_HDR_LEN);
		set_timeout(TMO_USB_WRITE);
		if (write(fd_bulk, buf, ACL_HDR_LEN + n) != ACL_HDR_LEN + n)
			err(1, "acl write");
		acl_credits--;
		off += n;
		pb = ACL_PB_CONT;
	}
}

/* L2CAP. */

static void
l2cap_send(struct chan *c, const uint8_t *d, int len)
{
	uint8_t pdu[L2CAP_PDU_MAX];

	if (c->state != CHAN_OPEN || len > (int)sizeof(pdu) - L2CAP_HDR_LEN)
		return;
	put_le16(pdu, len);
	put_le16(pdu + 2, c->remote_cid);
	memcpy(pdu + L2CAP_HDR_LEN, d, len);
	/* Audio may be dropped by the flush timeout; control traffic may not. */
	acl_send(pdu, len + L2CAP_HDR_LEN, c == &channels[CH_MEDIA]);
}

static void
sig_send(uint8_t code, uint8_t id, const uint8_t *d, int len)
{
	uint8_t pdu[L2CAP_SIG_MAX];
	int hdr;

	hdr = L2CAP_HDR_LEN + L2CAP_CMD_HDR_LEN;

	if (len > (int)sizeof(pdu) - hdr)
		return;
	put_le16(pdu, len + L2CAP_CMD_HDR_LEN);
	put_le16(pdu + 2, L2CAP_CID_SIGNALLING);
	pdu[4] = code;
	pdu[5] = id;
	put_le16(pdu + 6, len);
	memcpy(pdu + hdr, d, len);
	trace("sig>", code, pdu + L2CAP_HDR_LEN, len + L2CAP_CMD_HDR_LEN);
	acl_send(pdu, len + hdr, 0);
}

static struct chan *
chan_by_cid(unsigned int cid)
{
	int i;

	for (i = 0; i < CH_COUNT; i++)
		if (channels[i].local_cid == cid)
			return &channels[i];
	return NULL;
}

/* read the options of a configuration request; -1 if malformed */
static int
config_options(struct chan *c, const uint8_t *opt, int len)
{
	int o, ol;

	for (o = 0; o < len; o += 2 + ol) {
		if (len - o < 2)
			return -1;
		ol = opt[o + 1];
		if (ol > len - o - 2)
			return -1;
		if ((opt[o] & L2CAP_OPT_TYPE_MASK) != L2CAP_OPT_MTU)
			continue;
		if (ol != L2CAP_OPT_MTU_LEN)
			return -1;
		c->out_mtu = get_le16(opt + o + 2);
		if (c->out_mtu < L2CAP_HDR_LEN)
			return -1;
		if (c->out_mtu > L2CAP_PDU_MAX - L2CAP_HDR_LEN)
			c->out_mtu = L2CAP_PDU_MAX - L2CAP_HDR_LEN;
	}
	return 0;
}

/* handle the signalling commands in one L2CAP PDU */
static void
sig_input(const uint8_t *p, int n)
{
	uint8_t r[16];
	struct chan *c;
	const uint8_t *d;
	uint8_t code, id;
	int len;

	while (n >= L2CAP_CMD_HDR_LEN) {
		code = p[0];
		id = p[1];
		len = get_le16(p + 2);
		d = p + L2CAP_CMD_HDR_LEN;
		if (L2CAP_CMD_HDR_LEN + len > n)
			break;
		trace("sig<", code, p, L2CAP_CMD_HDR_LEN + len);
		switch (code) {
		case L2CAP_CMD_CONN_RSP:
			c = len < 8 ? NULL : chan_by_cid(get_le16(d + 2));
			if (c == NULL || c->state != CHAN_CONNECTING)
				break;
			c->result = get_le16(d + 4);
			if (c->result == L2CAP_CONN_OK) {
				c->remote_cid = get_le16(d);
				c->state = c->remote_cid >= L2CAP_CID_FIRST ?
				    CHAN_OPEN : CHAN_FAILED;
			} else if (c->result != L2CAP_CONN_PENDING)
				c->state = CHAN_FAILED;
			break;
		case L2CAP_CMD_CONFIG_REQ:
			c = len < 4 ? NULL : chan_by_cid(get_le16(d));
			if (c == NULL || c->state != CHAN_OPEN ||
			    config_options(c, d + 4, len - 4) == -1)
				break;
			put_le16(r, c->remote_cid);
			put_le16(r + 2, 0);		/* flags */
			put_le16(r + 4, L2CAP_CONFIG_OK);
			sig_send(L2CAP_CMD_CONFIG_RSP, id, r, 6);
			c->config_in = 1;
			break;
		case L2CAP_CMD_CONFIG_RSP:
			c = len < 6 ? NULL : chan_by_cid(get_le16(d));
			if (c == NULL || c->state != CHAN_OPEN)
				break;
			if (get_le16(d + 4) == L2CAP_CONFIG_OK)
				c->config_out = 1;
			else
				c->state = CHAN_FAILED;
			break;
		case L2CAP_CMD_DISCONN_REQ:
			if (len < 4)
				break;
			memcpy(r, d, 4);
			sig_send(L2CAP_CMD_DISCONN_RSP, id, r, 4);
			if ((c = chan_by_cid(get_le16(d))) != NULL)
				c->state = CHAN_FAILED;
			break;
		case L2CAP_CMD_CONN_REQ:	/* refuse, AVRCP for one */
			if (len < 4)
				break;
			put_le16(r, 0);
			put_le16(r + 2, get_le16(d + 2));
			put_le16(r + 4, L2CAP_CONN_BAD_PSM);
			put_le16(r + 6, 0);
			sig_send(L2CAP_CMD_CONN_RSP, id, r, 8);
			break;
		case L2CAP_CMD_ECHO_REQ:
			sig_send(L2CAP_CMD_ECHO_RSP, id, d,
			    len > L2CAP_ECHO_MAX ? L2CAP_ECHO_MAX : len);
			break;
		case L2CAP_CMD_INFO_REQ:
			if (len < 2)
				break;
			memcpy(r, d, 2);
			put_le16(r + 2, L2CAP_INFO_UNSUPPORTED);
			sig_send(L2CAP_CMD_INFO_RSP, id, r, 4);
			break;
		case L2CAP_CMD_ECHO_RSP:
			if (id == L2CAP_ECHO_IDENT &&
			    len == sizeof(echo_payload) &&
			    memcmp(d, echo_payload, len) == 0)
				echo_reply_ready = 1;
			break;
		case L2CAP_CMD_REJECT:
		case L2CAP_CMD_DISCONN_RSP:
		case L2CAP_CMD_INFO_RSP:
			break;
		default:
			put_le16(r, L2CAP_REJECT_NOT_UNDERSTOOD);
			sig_send(L2CAP_CMD_REJECT, id, r, 2);
		}
		p += L2CAP_CMD_HDR_LEN + len;
		n -= L2CAP_CMD_HDR_LEN + len;
	}
}

/* A complete L2CAP PDU from the device. */
static void
l2cap_input(const uint8_t *pdu, int len)
{
	struct chan *sig;
	const uint8_t *msg;
	uint8_t rej[AVDTP_HDR_LEN];
	unsigned int cid;
	int plen;

	sig = &channels[CH_SIGNALLING];
	msg = pdu + L2CAP_HDR_LEN;
	plen = len - L2CAP_HDR_LEN;

	if (len < L2CAP_HDR_LEN || get_le16(pdu) != (unsigned int)plen)
		return;
	cid = get_le16(pdu + 2);
	if (cid == L2CAP_CID_SIGNALLING) {
		sig_input(msg, plen);
		return;
	}
	if (cid != sig->local_cid || sig->state != CHAN_OPEN ||
	    plen < AVDTP_HDR_LEN)
		return;
	trace("avdtp<", cid, msg, plen);
	if ((msg[0] & AVDTP_MSG_MASK) == AVDTP_MSG_COMMAND) {
		/* we have no endpoints of our own: general reject */
		rej[0] = (msg[0] & AVDTP_LABEL_MASK) | AVDTP_MSG_GENERAL_REJECT;
		rej[1] = msg[1];
		l2cap_send(sig, rej, sizeof(rej));
	} else if (plen <= (int)sizeof(av_rsp)) {
		memcpy(av_rsp, msg, plen);
		av_rsp_len = plen;
		av_reply_ready = 1;
	}
}

static const char *
conn_failure(struct chan *c)
{
	switch (c->result) {
	case L2CAP_CONN_PENDING:
		return "still pending (authorization?)";
	case L2CAP_CONN_BAD_PSM:
		return "PSM not supported";
	case L2CAP_CONN_SECURITY:
		return "security block (link not encrypted or authenticated)";
	case L2CAP_CONN_NO_RESOURCES:
		return "no resources";
	}
	return c->state == CHAN_FAILED ? "refused" : "no response";
}

/* open channel idx to psm and run the configuration both ways */
static int
l2cap_connect(int idx, unsigned int psm)
{
	static uint8_t id = 1;
	struct chan *c;
	uint8_t d[8];
	int64_t end;
	int att;

	c = &channels[idx];
	memset(c, 0, sizeof(*c));
	c->local_cid = L2CAP_CID_FIRST + idx;
	c->out_mtu = L2CAP_MTU;
	c->state = CHAN_CONNECTING;
	put_le16(d, psm);
	put_le16(d + 2, c->local_cid);

	/* ask up to three times, unless the device says it is working */
	for (att = 0; att < 3 && c->state == CHAN_CONNECTING; att++) {
		sig_send(L2CAP_CMD_CONN_REQ, id++, d, 4);
		end = now_ms() + TMO_L2CAP_REQUEST;
		while (c->state == CHAN_CONNECTING && !quit &&
		    now_ms() < end)
			pump(TMO_POLL);
		if (c->state == CHAN_CONNECTING &&
		    c->result == L2CAP_CONN_PENDING) {
			end = now_ms() + TMO_L2CAP_PENDING;
			while (c->state == CHAN_CONNECTING && !quit &&
			    now_ms() < end)
				pump(TMO_POLL);
			break;
		}
	}
	if (c->state != CHAN_OPEN) {
		qwarnx("L2CAP connect to PSM 0x%04x failed: %s (result %d)",
		    psm, conn_failure(c), c->result);
		return -1;
	}

	put_le16(d, c->remote_cid);
	put_le16(d + 2, 0);			/* flags */
	d[4] = L2CAP_OPT_MTU;
	d[5] = L2CAP_OPT_MTU_LEN;
	put_le16(d + 6, L2CAP_MTU);
	sig_send(L2CAP_CMD_CONFIG_REQ, id++, d, 8);
	end = now_ms() + TMO_L2CAP_CONFIG;
	while (!(c->config_in && c->config_out) && c->state == CHAN_OPEN &&
	    !quit && now_ms() < end)
		pump(TMO_POLL);
	if (verbose)
		qfprintf(stderr, "L2CAP PSM 0x%04x: cid 0x%04x/0x%04x, "
		    "config in %d out %d\n", psm, c->local_cid,
		    c->remote_cid, c->config_in, c->config_out);
	return c->config_in && c->config_out && c->state == CHAN_OPEN ?
	    0 : -1;
}

/* HCI events and the receive pump. */

static void
inquiry_result_event(const uint8_t *p, int n)
{
	const uint8_t *addr, *cod;
	int i, j, num;

	if (n < 1)
		return;
	num = p[0];
	if (n < 1 + num * HCI_INQ_ENTRY_LEN)
		return;
	for (i = 0; i < num && inquiry_count < MAX_INQUIRY_RESULTS; i++) {
		addr = p + 1 + i * BDADDR_LEN;
		cod = p + 1 + num * HCI_INQ_ADDR_PSRM_RSV + i * HCI_COD_LEN;
		for (j = 0; j < inquiry_count; j++)
			if (memcmp(inquiry_results[j].bdaddr, addr,
			    BDADDR_LEN) == 0)
				break;
		if (j < inquiry_count)
			continue;
		memcpy(inquiry_results[j].bdaddr, addr, BDADDR_LEN);
		inquiry_results[j].class_of_device = cod[0] | cod[1] << 8 |
		    cod[2] << 16;
		inquiry_count++;
	}
}

static void
link_key_request(const uint8_t *addr)
{
	uint8_t r[BDADDR_LEN + HCI_LINK_KEY_LEN];

	memcpy(r, addr, BDADDR_LEN);
	if (key_load(addr, r + BDADDR_LEN))
		hci_cmd(HCI_CMD_LINK_KEY_REPLY, r, sizeof(r));
	else
		hci_cmd(HCI_CMD_LINK_KEY_NEG, r, BDADDR_LEN);
	explicit_bzero(r, sizeof(r));
}

/* Events that concern the device we talk to; the rest is ignored. */
static int
is_remote(const uint8_t *p, int n)
{
	return n >= BDADDR_LEN && memcmp(p, remote_bdaddr, BDADDR_LEN) == 0;
}

static int
is_acl_handle(const uint8_t *p)
{
	return connected && (get_le16(p) & ACL_HANDLE_MASK) == acl_handle;
}

static int
numeric_comparison_capable(int io)
{
	return io == HCI_IO_DISPLAY_YES_NO ||
	    io == HCI_IO_KEYBOARD_DISPLAY;
}

static void
handle_event(int code, const uint8_t *p, int n)
{
	uint8_t r[BDADDR_LEN + 3];
	int i, num, op;

	switch (code) {
	case HCI_EVT_INQUIRY_DONE:
		inquiry_done = 1;
		break;
	case HCI_EVT_INQUIRY_RESULT:
		inquiry_result_event(p, n);
		break;
	case HCI_EVT_CONN_COMPLETE:
		if (n < HCI_CONN_COMPLETE_LEN ||
		    !is_remote(p + HCI_CONN_ADDR_OFF, n))
			break;
		conn_status = p[0];
		if (p[0] == HCI_STATUS_OK) {
			acl_handle = get_le16(p + 1) & ACL_HANDLE_MASK;
			connected = 1;
		}
		connection_done = 1;
		break;
	case HCI_EVT_DISCONN:
		if (n >= HCI_DISCONN_LEN && is_acl_handle(p + 1)) {
			connected = 0;
			qfprintf(stderr, "link down (reason 0x%02x)\n", p[3]);
		}
		break;
	case HCI_EVT_AUTH_COMPLETE:
		if (n < 3 || !is_acl_handle(p + 1))
			break;
		auth_status = p[0];
		authentication_done = 1;
		break;
	case HCI_EVT_REMOTE_NAME:
		if (n < 1 + BDADDR_LEN || !is_remote(p + 1, n - 1))
			break;
		i = n - 1 - BDADDR_LEN;
		if (i > HCI_NAME_LEN)
			i = HCI_NAME_LEN;
		memset(remote_name, 0, sizeof(remote_name));
		memcpy(remote_name, p + 1 + BDADDR_LEN, i);
		remote_name_done = 1;
		break;
	case HCI_EVT_ENCRYPT_CHANGE:
		if (n < 4 || !is_acl_handle(p + 1))
			break;
		if (p[0] != HCI_STATUS_OK)
			enc_status = p[0];
		else if (p[3] == 0)
			enc_status = 0x100;
		else
			enc_status = HCI_STATUS_OK;
		encryption_done = 1;
		break;
	case HCI_EVT_CMD_COMPLETE:
		if (n < 4)
			break;
		op = get_le16(p + 1);
		/* the Intel bootloader acks download fragments as opcode 0 */
		if (op == HCI_CMD_DELETE_STORED_KEY) {
			if (n < 6)
				key_delete_status = -1;
			else
				key_delete_status = p[3];
			key_delete_done = 1;
			break;
		}
		if (op != waiting_opcode &&
		    !(op == 0 && waiting_opcode == INTEL_CMD_SECURE_SEND))
			break;
		command_status = p[3];
		command_reply_len = n - 3 < CMD_RET_MAX ? n - 3 : CMD_RET_MAX;
		memcpy(command_reply, p + 3, command_reply_len);
		command_done = 1;
		break;
	case HCI_EVT_CMD_STATUS:
		if (n < 4 || get_le16(p + 2) != (unsigned int)waiting_opcode)
			break;
		command_status = p[0];
		command_done = 1;
		break;
	case HCI_EVT_NUM_COMPLETED:
		if (n < 1 || n < 1 + 4 * p[0])
			break;
		num = p[0];
		for (i = 0; i < num; i++) {
			if (!is_acl_handle(p + 1 + 2 * i))
				continue;
			acl_credits += get_le16(p + 1 + 2 * num + 2 * i);
			if (acl_credits > acl_credits_max)
				acl_credits = acl_credits_max;
		}
		break;
	case HCI_EVT_LINK_KEY_REQ:
		if (is_remote(p, n))
			link_key_request(p);
		break;
	case HCI_EVT_LINK_KEY_NOTIFY:
		if (n < BDADDR_LEN + HCI_LINK_KEY_LEN + 1 || !is_remote(p, n))
			break;
		op = p[BDADDR_LEN + HCI_LINK_KEY_LEN];
		if ((op == HCI_KEY_AUTH_COMBINATION ||
		    op == HCI_KEY_AUTH_COMBINATION_P256) &&
		    numeric_comparison_capable(remote_io_capability)) {
			if (key_save(p, p + BDADDR_LEN, op))
				qfprintf(stderr, "paired with MITM authentication; "
				    "link key saved\n");
			else
				qwarnx("paired with MITM authentication; "
				    "link key was not saved");
		} else if (allow_just_works &&
		    (op == HCI_KEY_UNAUTH_COMBINATION ||
		    op == HCI_KEY_UNAUTH_COMBINATION_P256)) {
			if (key_save(p, p + BDADDR_LEN, op))
				qwarnx("paired without MITM protection (Just Works); "
				    "link key saved");
			else
				qwarnx("paired without MITM protection (Just Works); "
				    "link key was not saved");
		} else {
			uint8_t del[BDADDR_LEN + 1];

			/* Do not leave a rejected key in the controller's store. */
			memcpy(del, p, BDADDR_LEN);
			del[BDADDR_LEN] = 0;
			rejected_link_key = 1;
			key_delete_done = 0;
			key_delete_status = -1;
			hci_cmd(HCI_CMD_DELETE_STORED_KEY, del, sizeof(del));
			explicit_bzero(del, sizeof(del));
			key_save(p, NULL, 0);
		}
		break;
	case HCI_EVT_IO_CAP_RSP:
		if (n < BDADDR_LEN + 3 || !is_remote(p, n))
			break;
		remote_io_capability = p[BDADDR_LEN];
		break;
	case HCI_EVT_IO_CAP_REQ:
		if (!is_remote(p, n))
			break;
		memcpy(r, p, BDADDR_LEN);
		r[6] = allow_just_works ? HCI_IO_NO_INPUT_OUTPUT :
		    HCI_IO_DISPLAY_YES_NO;
		r[7] = HCI_OOB_NONE;
		r[8] = allow_just_works ? HCI_AUTH_BONDING_NOMITM :
		    HCI_AUTH_BONDING_MITM;
		hci_cmd(HCI_CMD_IO_CAP_REPLY, r, 9);
		explicit_bzero(r, sizeof(r));
		break;
	case HCI_EVT_USER_PASSKEY_REQ:
		if (n < BDADDR_LEN || !is_remote(p, n))
			break;
		/* No passkey-entry path is implemented. */
		hci_cmd(HCI_CMD_PASSKEY_NEG, p, BDADDR_LEN);
		errx(1, "refusing passkey-entry pairing; use a device "
		    "that supports Numeric Comparison");
		break;
	case HCI_EVT_USER_PASSKEY_NOTIFY:
		if (n < BDADDR_LEN + 4 || !is_remote(p, n))
			break;
		/* Never silently proceed with the passkey-entry model. */
		errx(1, "refusing passkey-notification pairing; use a device "
		    "that supports Numeric Comparison");
		break;
	case HCI_EVT_USER_CONFIRM:
		if (n < BDADDR_LEN + 4 || !is_remote(p, n))
			break;
		if (allow_just_works) {
			/* Just Works has no MITM check; explicitly opted in. */
			hci_cmd(HCI_CMD_USER_CONFIRM, p, BDADDR_LEN);
			break;
		}
		if (!numeric_comparison_capable(remote_io_capability)) {
			hci_cmd(HCI_CMD_USER_CONFIRM_NEG, p, BDADDR_LEN);
			errx(1, "refusing numeric comparison: incompatible "
			    "pairing capabilities");
		}
		if (!confirm_numeric(p + BDADDR_LEN)) {
			hci_cmd(HCI_CMD_USER_CONFIRM_NEG, p, BDADDR_LEN);
			errx(1, "numeric comparison rejected; pairing aborted");
		}
		hci_cmd(HCI_CMD_USER_CONFIRM, p, BDADDR_LEN);
		break;
	case HCI_EVT_SSP_COMPLETE:
		if (n < 1 + BDADDR_LEN || !is_remote(p + 1, n - 1))
			break;
		if (p[0])
			qfprintf(stderr, "pairing failed (status 0x%02x)\n",
			    p[0]);
		break;
	case HCI_EVT_VENDOR:
		if (n < 2)
			break;
		if (p[0] == INTEL_EVT_SECURE_RESULT) {
			vendor_secure_status = p[1];
			vendor_secure_done = 1;
		} else if (p[0] == INTEL_EVT_BOOTUP)
			vendor_boot_done = 1;
		break;
	}
}

/* Glue ACL fragments back into one L2CAP PDU. */

static void
acl_input(const uint8_t *p, int n)
{
	static uint8_t buf[L2CAP_PDU_MAX];
	static int have, need;
	int pb, len;

	if (n < ACL_HDR_LEN || !is_acl_handle(p)) {
		have = need = 0;
		return;
	}
	pb = (get_le16(p) >> ACL_PB_SHIFT) & ACL_PB_MASK;
	trace("acl<", 0, p, n < 48 ? n : 48);
	p += ACL_HDR_LEN;
	len = n - ACL_HDR_LEN;

	if (pb == ACL_PB_START_FLUSH || pb == ACL_PB_START_NOFLUSH) {
		have = need = 0;
		if (len < L2CAP_HDR_LEN)
			return;
		need = L2CAP_HDR_LEN + get_le16(p);
		if (need > (int)sizeof(buf) || len > need) {
			need = 0;
			return;
		}
		memcpy(buf, p, len);
		have = len;
	} else if (pb == ACL_PB_CONT && have > 0 && len > 0 &&
	    len <= need - have) {
		memcpy(buf + have, p, len);
		have += len;
	} else {
		have = need = 0;
		return;
	}
	if (have == need) {
		l2cap_input(buf, need);
		have = need = 0;
	}
}

/* take whole ACL packets out of bulk_buf; the caller holds the lock */
static void
bulk_parse(void)
{
	int l;

	while (bulk_len >= ACL_HDR_LEN) {
		l = ACL_HDR_LEN + get_le16(bulk_buf + 2);
		if (l > ACL_HDR_LEN + ACL_DATA_MAX) {
			bulk_len = 0;
			return;
		}
		if (bulk_len < l)
			break;
		acl_input(bulk_buf, l);
		memmove(bulk_buf, bulk_buf + l, bulk_len - l);
		bulk_len -= l;
	}
}

/* wait up to ms for the controller, then handle events and ACL data */
static void
pump(int ms)
{
	struct pollfd pfd[2];
	uint8_t tmp[1024];
	ssize_t n;
	int l, nfd;

	nfd = 1;

	if (pump_active)
		return;
	pump_active = 1;
	pfd[0].fd = fd_event;
	pfd[0].events = POLLIN;
	if (wake_pipe[0] != -1) {
		pfd[1].fd = wake_pipe[0];
		pfd[1].events = POLLIN;
		nfd = 2;
	}
	if (poll(pfd, nfd, ms) == -1 && errno != EINTR)
		err(1, "poll");
	if (nfd == 2)
		while (read(wake_pipe[0], tmp, sizeof(tmp)) > 0)
			;

	while ((n = read(fd_event, tmp, sizeof(tmp))) > 0) {
		if (event_len + n > (int)sizeof(event_buf)) {
			explicit_bzero(event_buf, sizeof(event_buf));
			event_len = 0;	/* overflow, start over */
		}
		memcpy(event_buf + event_len, tmp, n);
		event_len += n;
	}
	while (event_len >= HCI_EVT_HDR_LEN &&
	    event_len >= HCI_EVT_HDR_LEN + event_buf[1]) {
		l = event_buf[1];
		trace("evt", event_buf[0], event_buf + HCI_EVT_HDR_LEN,
		    event_buf[0] == HCI_EVT_LINK_KEY_NOTIFY ? BDADDR_LEN : l);
		handle_event(event_buf[0], event_buf + HCI_EVT_HDR_LEN, l);
		l += HCI_EVT_HDR_LEN;
		{
			int old_len = event_len;

			memmove(event_buf, event_buf + l, event_len - l);
			event_len -= l;
			explicit_bzero(event_buf + event_len, old_len - event_len);
		}
	}

	if (bulk_thread_started) {
		pthread_mutex_lock(&bulk_mutex);
		bulk_parse();
		pthread_mutex_unlock(&bulk_mutex);
	} else if (!bulk_raw_mode) {
		set_timeout(2);
		while ((n = read(fd_bulk, tmp, sizeof(tmp))) > 0) {
			trace("bulk<", 0, tmp, n < 64 ? n : 64);
			if (bulk_len + n > (int)sizeof(bulk_buf))
				bulk_len = 0;
			memcpy(bulk_buf + bulk_len, tmp, n);
			bulk_len += n;
		}
		bulk_parse();
	}
	explicit_bzero(tmp, sizeof(tmp));
	pump_active = 0;
}

/* AVDTP: one command at a time, waiting for the matching reply. */

/* Return the length of the reply parameters, or -1. */

static int
av_cmd(uint8_t sig, const uint8_t *p, int plen, uint8_t *out)
{
	static uint8_t label;
	uint8_t buf[64];
	int n;

	if (plen > (int)sizeof(buf) - AVDTP_HDR_LEN)
		return -1;
	label = (label + 1) & 0x0f;
	buf[0] = AVDTP_LABEL(label) | AVDTP_MSG_COMMAND;
	buf[1] = sig;
	if (plen > 0)
		memcpy(buf + AVDTP_HDR_LEN, p, plen);
	av_reply_ready = 0;
	trace("avdtp>", sig, buf, AVDTP_HDR_LEN + plen);
	l2cap_send(&channels[CH_SIGNALLING], buf, AVDTP_HDR_LEN + plen);
	if (!wait_flag(&av_reply_ready, TMO_AVDTP))
		return -1;
	if (av_rsp[0] >> 4 != label || av_rsp[1] != sig ||
	    (av_rsp[0] & AVDTP_PKT_MASK) != 0 ||
	    (av_rsp[0] & AVDTP_MSG_MASK) != AVDTP_MSG_ACCEPT) {
		if (verbose)
			trace("avdtp!", sig, av_rsp, av_rsp_len);
		return -1;
	}
	n = av_rsp_len - AVDTP_HDR_LEN;
	if (out != NULL && n > 0)
		memcpy(out, av_rsp + AVDTP_HDR_LEN, n);
	return n;
}

/*
 * Intel controllers boot into a bootloader that does nothing until the
 * host sends it a signed .sfi image through the vendor "secure send"
 * command and then resets it into the new firmware.
 */

/* Return the image type (INTEL_IMG_*) from Read Version, or -1. */

static int
intel_version(int show)
{
	uint8_t arg;
	int i, j, st, len, img;
	uint8_t *r;

	arg = INTEL_TLV_FORMAT_ARG;
	len = 0;
	img = -1;
	r = command_reply;

	if ((st = hci_cmd_wait(INTEL_CMD_READ_VERSION, &arg, 1))) {
		qwarnx("intel read version failed (status 0x%02x)", st);
		return -1;
	}
	len = command_reply_len;
	for (i = 1; i + 2 <= len && i + 2 + r[i + 1] <= len;
	    i += 2 + r[i + 1]) {
		if (show) {
			qprintf("tlv %02x len %d:", r[i], r[i + 1]);
			for (j = 0; j < r[i + 1]; j++)
				qprintf(" %02x", r[i + 2 + j]);
			qprintf("\n");
		}
		if (r[i] == INTEL_TLV_IMAGE_TYPE && r[i + 1] >= 1)
			img = r[i + 2];
	}
	if (img >= 0 || len != INTEL_LEGACY_VER_LEN) {
		if (img < 0 && show) {
			qprintf("no image type in the reply (%d bytes):", len);
			for (j = 0; j < len; j++)
				qprintf(" %02x", r[j]);
			qprintf("\n");
		}
		return img;
	}

	/* older parts use a fixed layout: status, platform, hw variant,
	 * hw revision, fw variant, fw revision, build, week, year, patch */
	if (show)
		qprintf("hw variant %u rev %u, fw variant %02x rev %u, build "
		    "%u week %u/%u, firmware likely ibt-%u-%u-%u.sfi\n",
		    r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[2], r[3],
		    r[5]);
	if (r[4] == INTEL_FW_BOOTLOADER)
		return INTEL_IMG_BOOTLOADER;
	if (r[4] == INTEL_FW_OPERATIONAL)
		return INTEL_IMG_OPERATIONAL;
	return -1;
}

static int
intel_boot_params(void)
{
	uint8_t *r;

	r = command_reply;
	if (hci_cmd_wait(INTEL_CMD_READ_BOOT, NULL, 0) ||
	    command_reply_len < INTEL_BOOT_LEN) {
		qwarnx("could not read the boot parameters");
		return -1;
	}
	qfprintf(stderr, "boot params: dev_revid %u secure_boot %u "
	    "key_type %u limited_cce %u\n", get_le16(r + INTEL_BOOT_DEV_REVID),
	    r[INTEL_BOOT_SECURE], r[INTEL_BOOT_KEY_TYPE],
	    r[INTEL_BOOT_LIMITED_CCE]);
	return r[INTEL_BOOT_SECURE] != 0;
}

/*
 * Send one fragment of at most INTEL_FRAG_MAX bytes and wait for the
 * bootloader to acknowledge it.  Like iwmbtfw we expect the ack on
 * bulk-in, but a command complete on the event endpoint will do too.
 */
static void
intel_fragment(uint8_t type, const uint8_t *d, size_t len)
{
	uint8_t p[1 + INTEL_FRAG_MAX], b[INTEL_BOOT_ACK_MAX];
	int64_t end;
	ssize_t n;

	p[0] = type;
	memcpy(p + 1, d, len);
	waiting_opcode = INTEL_CMD_SECURE_SEND;
	command_done = 0;
	hci_cmd(INTEL_CMD_SECURE_SEND, p, len + 1);

	end = now_ms() + TMO_INTEL_ACK;
	for (;;) {
		set_timeout(TMO_POLL);
		if ((n = read(fd_bulk, b, sizeof(b))) > 0) {
			trace("ack(bulk)", 0, b, n);
			break;
		}
		pump(1);
		if (command_done)
			break;
		if (vendor_secure_done && vendor_secure_status)
			errx(1, "the controller rejected the firmware "
			    "(result 0x%02x)", vendor_secure_status);
		if (quit)
			exit(1);
		if (now_ms() > end)
			errx(1, "no acknowledgment for download fragment %02x",
			    type);
	}
	waiting_opcode = -1;
}

static uint8_t *
firmware_read(const char *path, size_t *sizep)
{
	struct stat st;
	uint8_t *fw;
	FILE *f;
	int fd;

	if ((fd = open(path, O_RDONLY | O_NOFOLLOW)) == -1)
		err(1, "%s", path);
	if (fstat(fd, &st) == -1)
		err(1, "%s", path);
	if (!S_ISREG(st.st_mode) || st.st_size > INTEL_FW_MAX_SIZE)
		errx(1, "%s: not a usable firmware file", path);
	*sizep = st.st_size;
	if ((fw = malloc(*sizep)) == NULL)
		err(1, NULL);
	if ((f = fdopen(fd, "r")) == NULL)
		err(1, "%s", path);
	if (fread(fw, 1, *sizep, f) != *sizep)
		errx(1, "%s: short read", path);
	fclose(f);
	return fw;
}

static void
intel_boot(const char *path, int ecdsa)
{
	static const uint8_t reset_boot[4] = INTEL_RESET_BOOT;
	static const uint8_t evt_mask[8] = INTEL_EVENT_MASK;
	uint8_t *fw, rp[INTEL_RESET_LEN];
	size_t size, sent, ready;
	size_t hdr;
	uint32_t boot;
	int boot_found;

	hdr = ecdsa ? INTEL_ECDSA_HDR_LEN : INTEL_RSA_HDR_LEN;
	boot = 0;
	boot_found = 0;

	fw = firmware_read(path, &size);
	if (size < hdr)
		errx(1, "%s: too small to be an Intel firmware file", path);

	if (intel_boot_params() != 1)
		errx(1, "Intel secure boot is not enabled");
	qfprintf(stderr, "loading %s (%zu bytes, %s header)\n", path, size,
	    ecdsa ? "ECDSA" : "RSA");
	vendor_secure_done = 0;
	bulk_raw_mode = 1;

	/* the signed header: CSS block, public key, signature */
	intel_fragment(INTEL_FRAG_CSS, fw, INTEL_CSS_LEN);
	if (ecdsa) {
		intel_fragment(INTEL_FRAG_PUBKEY, fw + INTEL_ECDSA_KEY_OFF,
		    INTEL_ECDSA_CHUNK);
		intel_fragment(INTEL_FRAG_SIGNATURE, fw + INTEL_ECDSA_SIG_OFF,
		    INTEL_ECDSA_CHUNK);
	} else {
		intel_fragment(INTEL_FRAG_PUBKEY, fw + INTEL_RSA_KEY1_OFF,
		    INTEL_RSA_CHUNK);
		intel_fragment(INTEL_FRAG_PUBKEY, fw + INTEL_RSA_KEY2_OFF,
		    INTEL_RSA_CHUNK);
		intel_fragment(INTEL_FRAG_SIGNATURE, fw + INTEL_RSA_SIG1_OFF,
		    INTEL_RSA_CHUNK);
		intel_fragment(INTEL_FRAG_SIGNATURE, fw + INTEL_RSA_SIG2_OFF,
		    INTEL_RSA_CHUNK);
	}

	/*
	 * The rest is a list of HCI commands.  They go out in chunks of at
	 * most INTEL_FRAG_MAX bytes, cut at a multiple of four.  Write Boot
	 * Params holds the address the final reset has to jump to.
	 */
	sent = hdr;
	ready = 0;
	while (size - sent - ready >= HCI_CMD_HDR_LEN) {
		const uint8_t *cmd = fw + sent + ready;

		if (cmd[2] > size - sent - ready - HCI_CMD_HDR_LEN)
			errx(1, "%s: truncated firmware file", path);
		if (get_le16(cmd) == INTEL_CMD_WRITE_BOOT && cmd[2] >= 4) {
			boot = cmd[3] | cmd[4] << 8 | cmd[5] << 16 |
			    (uint32_t)cmd[6] << 24;
			boot_found = 1;
		}
		ready += HCI_CMD_HDR_LEN + cmd[2];
		while (ready >= INTEL_FRAG_MAX) {
			intel_fragment(INTEL_FRAG_PAYLOAD, fw + sent,
			    INTEL_FRAG_MAX);
			sent += INTEL_FRAG_MAX;
			ready -= INTEL_FRAG_MAX;
		}
		if (ready > 0 && ready % INTEL_FRAG_ALIGN == 0) {
			intel_fragment(INTEL_FRAG_PAYLOAD, fw + sent, ready);
			sent += ready;
			ready = 0;
		}
	}
	bulk_raw_mode = 0;
	if (!boot_found)
		errx(1, "%s: no boot address in the firmware", path);
	if (sent + ready != size)
		qwarnx("%zu trailing bytes not sent", size - sent);
	free(fw);

	if (!wait_flag(&vendor_secure_done, TMO_INTEL_DONE))
		errx(1, "the controller never confirmed the download");
	if (vendor_secure_status)
		errx(1, "the controller rejected the firmware (result 0x%02x)",
		    vendor_secure_status);

	/* reset into the new firmware, at the address from the file */
	memcpy(rp, reset_boot, sizeof(reset_boot));
	rp[4] = boot;
	rp[5] = boot >> 8;
	rp[6] = boot >> 16;
	rp[7] = boot >> 24;
	vendor_boot_done = 0;
	hci_cmd(INTEL_CMD_RESET, rp, INTEL_RESET_LEN);
	if (wait_flag(&vendor_boot_done, TMO_INTEL_BOOT))
		qfprintf(stderr, "firmware running\n");
	else
		qwarnx("no boot event; if the device came back under a new "
		    "ugen unit, run again with that unit");
	pump(300);

	/* iwmbtfw resets the USB device here, we reopen the endpoints */
	usb_reopen();
	hci_cmd_try(INTEL_CMD_SET_EVT_MASK, evt_mask, sizeof(evt_mask),
	    TMO_CMD_SHORT);
}

/* send the controller back to its bootloader; it shows up again */
static void
intel_reset(void)
{
	static const uint8_t rp[INTEL_RESET_LEN] = INTEL_RESET_BOOTLOADER;

	hci_cmd(INTEL_CMD_RESET, rp, INTEL_RESET_LEN);
	pump(500);
	qfprintf(stderr, "reset sent; wait a few seconds and check dmesg for "
	    "the ugen unit it comes back as\n");
}

static void
intel_setup(const char *fw, int ecdsa, int info)
{
	int img;

	img = intel_version(info);
	if (info) {
		intel_boot_params();
		return;
	}
	if (img == INTEL_IMG_OPERATIONAL)
		return;
	if (img != INTEL_IMG_BOOTLOADER)
		errx(1, "unexpected Intel image type %d, try btspk info", img);
	if (fw == NULL)
		errx(1, "the controller needs firmware: btspk -f ibt-XX.sfi");
	intel_boot(fw, ecdsa);
}

/* Upload firmware as a standalone controller operation. */
static void
upload(const char *fw, int ecdsa)
{
	int img;

	if (fw == NULL)
		errx(1, "upload requires a firmware file: "
		    "btspk -f ibt-XX.sfi upload");
	img = intel_version(0);
	if (img != INTEL_IMG_BOOTLOADER)
		errx(1, "the controller is not in the bootloader "
		    "(image type %d)", img);
	intel_boot(fw, ecdsa);
}


static void
hci_init(void)
{
	uint8_t p[HCI_EVT_MASK_LEN];
	uint64_t m;
	int i;

	bulk_start();
	if (hci_cmd_wait(HCI_CMD_RESET, NULL, 0) != HCI_STATUS_OK)
		errx(1, "controller reset failed");

	m = HCI_EVT_MASK_DEFAULT | HCI_EVT_BIT(HCI_EVT_IO_CAP_REQ) |
	    HCI_EVT_BIT(HCI_EVT_IO_CAP_RSP) |
	    HCI_EVT_BIT(HCI_EVT_USER_CONFIRM) |
	    HCI_EVT_BIT(HCI_EVT_USER_PASSKEY_REQ) |
	    HCI_EVT_BIT(HCI_EVT_USER_PASSKEY_NOTIFY) |
	    HCI_EVT_BIT(HCI_EVT_SSP_COMPLETE);
	for (i = 0; i < HCI_EVT_MASK_LEN; i++)
		p[i] = m >> (8 * i);
	if (hci_cmd_wait(HCI_CMD_SET_EVENT_MASK, p, HCI_EVT_MASK_LEN) !=
	    HCI_STATUS_OK)
		errx(1, "could not enable required HCI events");

	p[0] = HCI_SSP_ENABLE;
	if (hci_cmd_wait(HCI_CMD_WRITE_SSP_MODE, p, 1) != HCI_STATUS_OK)
		errx(1, "controller refused Secure Simple Pairing; "
		    "refusing to continue in insecure mode");

	p[0] = 1;
	if (hci_cmd_wait(HCI_CMD_WRITE_SECURE_CONNECTIONS, p, 1) !=
	    HCI_STATUS_OK)
		errx(1, "controller refused Secure Connections; refusing to "
		    "continue in insecure mode");

	if (hci_cmd_wait(HCI_CMD_READ_BUF_SIZE, NULL, 0) != HCI_STATUS_OK ||
	    command_reply_len < 7)
		errx(1, "could not read valid controller buffer sizes");
	acl_mtu = get_le16(command_reply + 1);
	acl_credits_max = get_le16(command_reply + 4);
	if (acl_mtu < HCI_ACL_MIN_LEN || acl_credits_max < 1)
		errx(1, "implausible buffer sizes from the controller");
	if (acl_mtu > ACL_DATA_MAX)
		acl_mtu = ACL_DATA_MAX;
	acl_credits = acl_credits_max;
}

static void
scan(void)
{
	static const uint8_t giac[HCI_LAP_LEN] = HCI_LAP_GIAC;
	struct inquiry_result *ir;
	uint8_t p[10];
	int i;

	memset(p, 0, sizeof(p));
	memcpy(p, giac, HCI_LAP_LEN);
	p[3] = HCI_INQUIRY_LEN;
	p[4] = HCI_INQUIRY_UNLIMITED;
	if (hci_cmd_wait(HCI_CMD_INQUIRY, p, 5))
		errx(1, "inquiry failed");
	qfprintf(stderr, "scanning...\n");
	wait_flag(&inquiry_done, TMO_INQUIRY);

	for (i = 0; i < inquiry_count; i++) {
		ir = &inquiry_results[i];
		memset(p, 0, sizeof(p));
		memcpy(p, ir->bdaddr, BDADDR_LEN);
		p[6] = HCI_PSRM_R1;
		memcpy(remote_bdaddr, ir->bdaddr, BDADDR_LEN);
		remote_name_done = 0;
		remote_name[0] = '\0';
		if (hci_cmd_wait(HCI_CMD_REMOTE_NAME, p, 10) == 0)
			wait_flag(&remote_name_done, TMO_NAME);
		qprintf("%s  class %06x%s  %s\n", bd_ntoa(ir->bdaddr),
		    ir->class_of_device,
		    HCI_COD_MAJOR(ir->class_of_device) == HCI_COD_MAJOR_AUDIO ?
		    " (audio)" : "", remote_name);
	}
	if (inquiry_count == 0)
		qprintf("no devices found; is the device in pairing mode?\n");
}

/* At exit, hang up politely, stop the reader and close the devices. */

static void
disconnect(void)
{
	uint8_t p[3];
	int i;

	if (disconnecting)
		return;
	disconnecting = 1;
	if (connected && (fd_usb_helper != -1 || fd_control != -1)) {
		put_le16(p, acl_handle);
		p[2] = HCI_REASON_USER_TERM;
		hci_cmd(HCI_CMD_DISCONNECT, p, 3);
		for (i = 0; i < 40 && connected; i++)
			pump(TMO_DISCONNECT_POLL);
	}
	if (bulk_thread_started) {
		pthread_cancel(bulk_thread);
		pthread_join(bulk_thread, NULL);
		bulk_thread_started = 0;
	}
	close(fd_event);
	close(fd_bulk);
	close(fd_control);
	if (fd_usb_helper != -1) {
		close(fd_usb_helper);
		fd_usb_helper = -1;
	}
	explicit_bzero(event_buf, sizeof(event_buf));
	explicit_bzero(bulk_buf, sizeof(bulk_buf));
	explicit_bzero(command_reply, sizeof(command_reply));
	explicit_bzero(remote_bdaddr, sizeof(remote_bdaddr));
	explicit_bzero(remote_name, sizeof(remote_name));
}

static void
check_rejected_key(void)
{
	if (!rejected_link_key)
		return;
	if (!wait_flag(&key_delete_done, TMO_CMD) || key_delete_status !=
	    HCI_STATUS_OK) {
		rejected_link_key = 0;
		errx(1, "could not remove rejected link key from controller");
	}
	rejected_link_key = 0;
	errx(1, "refusing an unauthenticated link key; "
	    "use -J to explicitly allow Just Works pairing");
}

static void
link_up(void)
{
	uint8_t p[13];

	memcpy(p, remote_bdaddr, BDADDR_LEN);
	put_le16(p + 6, HCI_PKT_TYPES_ACL);
	p[8] = HCI_PSRM_R1;
	p[9] = 0;
	put_le16(p + 10, 0);			/* clock offset */
	p[12] = HCI_NO_ROLE_SWITCH;
	connection_done = 0;
	if (hci_cmd_wait(HCI_CMD_CREATE_CONN, p, 13))
		errx(1, "create connection refused");
	if (!wait_flag(&connection_done, TMO_CONNECT))
		errx(1, "the device did not answer; is it on and in range?");
	if (conn_status)
		errx(1, "connection failed (status 0x%02x)", conn_status);

	if (skip_security) {
		pump(TMO_SETTLE);
		return;
	}

	/* Require authenticated Secure Simple Pairing on new bonds. */
	remote_io_capability = -1;
	put_le16(p, acl_handle);
	authentication_done = 0;
	if (hci_cmd_wait(HCI_CMD_AUTH_REQUESTED, p, 2))
		errx(1, "authentication request refused");
	if (!wait_flag(&authentication_done, TMO_AUTH))
		errx(1, "authentication timed out");
	if (auth_status) {
		if (auth_status == HCI_STATUS_KEY_MISSING)
			key_save(remote_bdaddr, NULL, 0);
		errx(1, "authentication failed (0x%02x): hold the device's "
		    "Bluetooth button until it blinks, then run again",
		    auth_status);
	}
	check_rejected_key();

	put_le16(p, acl_handle);
	p[2] = HCI_ENCRYPT_ON;
	encryption_done = 0;
	if (hci_cmd_wait(HCI_CMD_SET_ENCRYPTION, p, 3))
		errx(1, "encryption request refused");
	if (!wait_flag(&encryption_done, TMO_ENCRYPT) || enc_status)
		errx(1, "could not turn encryption on");
	check_rejected_key();

	if (hci_cmd_wait(HCI_CMD_READ_ENCRYPTION_KEY_SIZE, p, 2) !=
	    HCI_STATUS_OK || command_reply_len < 4 ||
	    command_reply[3] < HCI_LINK_KEY_LEN)
		errx(1, "encryption key is shorter than 128 bits");
	pump(TMO_LINK_SETUP);
	check_rejected_key();
}

/* let the controller drop audio it cannot deliver in time */
static void
set_flush_timeout(void)
{
	uint8_t p[4];

	if (flush_ms == 0)
		return;
	put_le16(p, acl_handle);
	put_le16(p + 2, HCI_SLOTS_PER_MS(flush_ms));
	hci_cmd_try(HCI_CMD_WRITE_FLUSH_TO, p, 4, TMO_CMD_SHORT);
}

/* A2DP */

/* find a free sink, read what it can do and pick a configuration */
static void
a2dp_negotiate(struct a2dp *a)
{
	uint8_t r[256], p[16];
	const uint8_t *caps = NULL;
	int n, i, caplen;

	if ((n = av_cmd(AVDTP_DISCOVER, NULL, 0, r)) < AVDTP_SEP_INFO_LEN)
		errx(1, "AVDTP discover failed");
	a->remote_seid = 0;
	for (i = 0; i + AVDTP_SEP_INFO_LEN <= n; i += AVDTP_SEP_INFO_LEN) {
		if ((r[i + 1] >> AVDTP_SEP_MEDIA_SHIFT) != AVDTP_MEDIA_AUDIO ||
		    !(r[i + 1] & AVDTP_SEP_TSEP_SINK) ||
		    (r[i] & AVDTP_SEP_INUSE))
			continue;
		a->remote_seid = r[i] >> AVDTP_SEID_SHIFT;
		break;
	}
	if (a->remote_seid == 0)
		errx(1, "no free A2DP sink on the device");

	p[0] = a->remote_seid << AVDTP_SEID_SHIFT;
	if ((n = av_cmd(AVDTP_GET_CAPABILITIES, p, 1, r)) < 0)
		errx(1, "AVDTP get capabilities failed");
	for (i = 0; n - i >= 2; i += 2 + caplen) {
		caplen = r[i + 1];
		if (caplen > n - i - 2)
			errx(1, "bad AVDTP capability length");
		if (r[i] == AVDTP_CAP_CODEC &&
		    caplen >= AVDTP_CODEC_INFO_LEN &&
		    r[i + 3] == AVDTP_CODEC_SBC) {
			caps = r + i + 4;
			break;
		}
	}
	if (caps == NULL)
		errx(1, "the device offers no SBC");

	/* every sink has to do 44.1 and 48 kHz, all modes, 16x8, loudness */
	if (caps[0] & A2DP_FREQ_44100) {
		a->sample_rate = 44100;
		a->codec_info[0] = A2DP_FREQ_44100;
	} else if (caps[0] & A2DP_FREQ_48000) {
		a->sample_rate = 48000;
		a->codec_info[0] = A2DP_FREQ_48000;
	} else
		errx(1, "no usable sample rate");
	if (caps[0] & A2DP_MODE_JOINT) {
		a->sbc_mode = SBC_MODE_JOINT_STEREO;
		a->codec_info[0] |= A2DP_MODE_JOINT;
	} else if (caps[0] & A2DP_MODE_STEREO) {
		a->sbc_mode = SBC_MODE_STEREO;
		a->codec_info[0] |= A2DP_MODE_STEREO;
	} else
		errx(1, "no stereo mode");
	if (!(caps[1] & A2DP_BLOCKS_16) || !(caps[1] & A2DP_SUBBANDS_8) ||
	    !(caps[1] & A2DP_ALLOC_LOUDNESS))
		errx(1, "the device lacks 16 blocks, 8 subbands or loudness");
	a->codec_info[1] = A2DP_BLOCKS_16 | A2DP_SUBBANDS_8 |
	    A2DP_ALLOC_LOUDNESS;
	if (caps[2] < A2DP_BITPOOL_MIN || caps[3] < caps[2])
		errx(1, "bad SBC bitpool range");
	a->sbc_bitpool = caps[3] < A2DP_BITPOOL_MAX ? caps[3] :
	    A2DP_BITPOOL_MAX;
	if (a->sbc_bitpool < caps[2])
		errx(1, "unusable SBC bitpool range");
	a->codec_info[2] = caps[2];
	a->codec_info[3] = a->sbc_bitpool;

	p[0] = a->remote_seid << AVDTP_SEID_SHIFT;
	p[1] = AVDTP_LOCAL_SEID << AVDTP_SEID_SHIFT;
	p[2] = AVDTP_CAP_TRANSPORT;
	p[3] = 0;
	p[4] = AVDTP_CAP_CODEC;
	p[5] = AVDTP_CODEC_INFO_LEN;
	p[6] = AVDTP_MEDIA_AUDIO;
	p[7] = AVDTP_CODEC_SBC;
	memcpy(p + 8, a->codec_info, 4);
	if (av_cmd(AVDTP_SET_CONFIGURATION, p, 12, NULL) < 0)
		errx(1, "the device rejected the configuration");
	p[0] = a->remote_seid << AVDTP_SEID_SHIFT;
	if (av_cmd(AVDTP_OPEN, p, 1, NULL) < 0)
		errx(1, "the device would not open the stream");
}

/* Capture audio from sndiod, encode it and send it until interrupted. */

static void
stream(struct a2dp *a, const char *dev, int nframes, int bufms)
{
	struct sio_hdl *hdl;
	struct sio_par par;
	sbc_t sbc;
	uint8_t *pcm, out[1024], pkt[ACL_DATA_MAX];
	size_t codesize, framelen, want, have;
	ssize_t n, written;
	uint32_t ts, ssrc;
	uint16_t seq;
	int nf, i;

	have = 0;
	ts = 0;
	ssrc = arc4random();
	seq = 0;

	if (sbc_init(&sbc, 0) == -1)
		errx(1, "sbc_init failed");
	sbc.frequency = a->sample_rate == 44100 ? SBC_FREQ_44100 :
	    SBC_FREQ_48000;
	sbc.blocks = SBC_BLK_16;
	sbc.subbands = SBC_SB_8;
	sbc.mode = a->sbc_mode;
	sbc.allocation = SBC_AM_LOUDNESS;
	sbc.bitpool = a->sbc_bitpool;
	sbc.endian = SBC_LE;
	codesize = sbc_get_codesize(&sbc);
	framelen = sbc_get_frame_length(&sbc);
	if (codesize == 0 || framelen == 0 || framelen > sizeof(out))
		errx(1, "unexpected SBC frame size");

	/* as many frames per packet as asked for and the MTU allows */
	nf = (channels[CH_MEDIA].out_mtu - MEDIA_HDR_LEN) / framelen;
	if (nf > (int)(sizeof(out) / framelen))
		nf = sizeof(out) / framelen;
	if (nf > nframes)
		nf = nframes;
	if (nf > MEDIA_MAX_FRAMES)
		nf = MEDIA_MAX_FRAMES;
	if (channels[CH_MEDIA].out_mtu <= MEDIA_HDR_LEN + framelen || nf < 1)
		errx(1, "the device's L2CAP MTU is too small");
	want = codesize * nf;
	if ((pcm = malloc(want)) == NULL)
		err(1, NULL);

	if ((hdl = sio_open(dev, SIO_REC, 0)) == NULL)
		errx(1, "cannot open sndio device %s", dev);
	if (pledge("stdio unix audio", NULL) == -1)
		err(1, "pledge stdio unix audio");
	sio_initpar(&par);
	par.bits = 16;
	par.sig = 1;
	par.le = 1;
	par.rchan = 2;
	par.rate = a->sample_rate;
	par.round = a->sample_rate * STREAM_BLOCK_MS / 1000;
	par.appbufsz = a->sample_rate * bufms / 1000;
	if (par.appbufsz < 2 * par.round)
		par.appbufsz = 2 * par.round;
	if (!sio_setpar(hdl, &par) || !sio_getpar(hdl, &par) ||
	    par.bits != 16 || par.sig != 1 || par.le != 1 ||
	    par.rchan != 2 || par.rate != (unsigned int)a->sample_rate)
		errx(1, "sndio will not give %d Hz 16 bit stereo",
		    a->sample_rate);
	if (!sio_start(hdl))
		errx(1, "sio_start failed");

	qfprintf(stderr, "streaming %d Hz, %d frames per packet, %u ms "
	    "buffer (Ctrl-C to stop)\n", a->sample_rate, nf,
	    par.appbufsz * 1000 / par.rate);

	while (!quit && connected) {
		n = sio_read(hdl, pcm + have, want - have);
		if (n == 0)
			break;			/* sndiod went away, or EINTR */
		have += n;
		if (have < want)
			continue;
		have = 0;

		for (i = 0; i < nf; i++)
			if (sbc_encode(&sbc, pcm + i * codesize, codesize,
			    out + i * framelen, framelen, &written) < 0 ||
			    written != (ssize_t)framelen)
				errx(1, "sbc_encode failed");

		pkt[0] = RTP_VERSION_2;
		pkt[1] = RTP_PT_DYNAMIC;
		put_be16(pkt + 2, seq++);
		put_be32(pkt + 4, ts);
		put_be32(pkt + 8, ssrc);
		pkt[RTP_HDR_LEN] = nf;		/* SBC payload header */
		memcpy(pkt + MEDIA_HDR_LEN, out, nf * framelen);
		ts += nf * A2DP_SAMPLES_PER_FRAME;
		l2cap_send(&channels[CH_MEDIA], pkt,
		    MEDIA_HDR_LEN + nf * framelen);
		pump(0);
	}
	sio_close(hdl);
	sbc_finish(&sbc);
	explicit_bzero(out, sizeof(out));
	explicit_bzero(pkt, sizeof(pkt));
	explicit_bzero(pcm, want);
	free(pcm);
}

static void
play(const char *addr, const char *dev, int nframes, int bufms)
{
	struct a2dp a;
	uint8_t p[1];

	if (bd_aton(addr, remote_bdaddr) == -1)
		errx(1, "%s: not a Bluetooth address", addr);
	link_up();
	set_flush_timeout();

	if (l2cap_connect(CH_SIGNALLING, PSM_AVDTP) == -1)
		errx(1, "the AVDTP signalling channel was refused");
	if (verbose) {
		echo_reply_ready = 0;
		sig_send(L2CAP_CMD_ECHO_REQ, L2CAP_ECHO_IDENT, echo_payload,
		    sizeof(echo_payload));
		wait_flag(&echo_reply_ready, TMO_ECHO_PROBE);
		qfprintf(stderr, "echo %s\n", echo_reply_ready ? "answered" :
		    "not answered");
	}

	a2dp_negotiate(&a);
	if (l2cap_connect(CH_MEDIA, PSM_AVDTP) == -1)
		errx(1, "the AVDTP media channel was refused");
	p[0] = a.remote_seid << AVDTP_SEID_SHIFT;
	if (av_cmd(AVDTP_START, p, 1, NULL) < 0)
		errx(1, "the device would not start the stream");
	stream(&a, dev, nframes, bufms);
	av_cmd(AVDTP_CLOSE, p, 1, NULL);
}

/* Confirm the six-digit Secure Simple Pairing numeric comparison. */
static int
confirm_numeric(const uint8_t *p)
{
	char line[16], prompt[128];
	uint32_t value;
	ssize_t n;
	int fd, i, len, ok;

	value = (uint32_t)p[0] | (uint32_t)p[1] << 8 |
	    (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
	if (value > 999999)
		return 0;
	if ((fd = open("/dev/tty", O_RDWR)) == -1)
		return 0;
	len = snprintf(prompt, sizeof(prompt),
	    "Bluetooth numeric comparison: %06u\n"
	    "Verify that the same number is shown on the device. "
	    "Accept? [y/N] ", value);
	if (len < 0 || len >= (int)sizeof(prompt) ||
	    write(fd, prompt, len) != len) {
		close(fd);
		return 0;
	}
	memset(line, 0, sizeof(line));
	for (i = 0; i < (int)sizeof(line) - 1; i++) {
		n = read(fd, line + i, 1);
		if (n != 1 || line[i] == '\n')
			break;
	}
	ok = i == 1 && (line[0] == 'y' || line[0] == 'Y') &&
	    (n == 1 && line[1] == '\n');
	explicit_bzero(line, sizeof(line));
	explicit_bzero(prompt, sizeof(prompt));
	close(fd);
	return ok;
}

/* Restrict filesystem access after USB setup and privilege dropping. */
static void
sandbox(const char *dev)
{
	if (unveil(key_dir, "rwc") == -1)
		err(1, "unveil %s", key_dir);
	if (unveil("/dev/tty", "rw") == -1)
		err(1, "unveil /dev/tty");
	/* sndio connects to sndiod through its Unix-domain socket. */
	if (unveil("/tmp/sndio", "w") == -1)
		err(1, "unveil /tmp/sndio (sndio device %s)", dev);
	if (unveil(NULL, NULL) == -1)
		err(1, "unveil lock");
}

/* Read or write an exact IPC record, retrying interrupted operations. */
static int
read_full(int fd, void *buf, size_t len)
{
	uint8_t *p = buf;
	ssize_t n;
	size_t done = 0;

	while (done < len) {
		n = read(fd, p + done, len - done);
		if (n == -1 && errno == EINTR)
			continue;
		if (n <= 0)
			return -1;
		done += n;
	}
	return 0;
}

static int
write_full(int fd, const void *buf, size_t len)
{
	const uint8_t *p = buf;
	ssize_t n;
	size_t done = 0;

	while (done < len) {
		n = write(fd, p + done, len - done);
		if (n == -1 && errno == EINTR)
			continue;
		if (n <= 0)
			return -1;
		done += n;
	}
	return 0;
}

static int
usb_helper_call(struct usb_helper_msg *msg)
{
	struct usb_helper_reply reply;

	if (fd_usb_helper == -1) {
		errno = EBADF;
		return -1;
	}
	if (write_full(fd_usb_helper, msg, sizeof(*msg)) == -1 ||
	    read_full(fd_usb_helper, &reply, sizeof(reply)) == -1) {
		explicit_bzero(msg, sizeof(*msg));
		errno = EPIPE;
		return -1;
	}
	explicit_bzero(msg, sizeof(*msg));
	if (reply.result == -1) {
		errno = reply.error;
		explicit_bzero(&reply, sizeof(reply));
		return -1;
	}
	explicit_bzero(&reply, sizeof(reply));
	return 0;
}

/* USB control requests are proxied after the main process is pledged. */
static int
usb_do_request(struct usb_ctl_request *ur)
{
	struct usb_helper_msg msg;
	int len;

	if (fd_usb_helper == -1)
		return ioctl(fd_control, USB_DO_REQUEST, ur);

	memset(&msg, 0, sizeof(msg));
	len = UGETW(ur->ucr_request.wLength);
	if (len > (int)sizeof(msg.data) ||
	    (len != 0 && ur->ucr_data == NULL)) {
		errno = EINVAL;
		return -1;
	}
	msg.op = USB_HELPER_REQUEST;
	msg.len = len;
	msg.ur = *ur;
	msg.ur.ucr_data = NULL;
	if (len != 0)
		memcpy(msg.data, ur->ucr_data, len);
	return usb_helper_call(&msg);
}

static int
usb_bulk_ioctl(unsigned long request, int *value)
{
	struct usb_helper_msg msg;

	if (fd_usb_helper == -1)
		return ioctl(fd_bulk, request, value);
	memset(&msg, 0, sizeof(msg));
	msg.op = USB_HELPER_BULK_IOCTL;
	msg.request = request;
	msg.value = *value;
	return usb_helper_call(&msg);
}

static int
usb_hci_command_allowed(uint16_t op, int len)
{
	switch (op) {
	case HCI_CMD_INQUIRY:
		return len == 5;
	case HCI_CMD_CREATE_CONN:
		return len == 13;
	case HCI_CMD_DISCONNECT:
		return len == 3;
	case HCI_CMD_LINK_KEY_REPLY:
		return len == BDADDR_LEN + HCI_LINK_KEY_LEN;
	case HCI_CMD_LINK_KEY_NEG:
		return len == BDADDR_LEN;
	case HCI_CMD_DELETE_STORED_KEY:
		return len == BDADDR_LEN + 1;
	case HCI_CMD_AUTH_REQUESTED:
		return len == 2;
	case HCI_CMD_SET_ENCRYPTION:
		return len == 3;
	case HCI_CMD_REMOTE_NAME:
		return len == 10;
	case HCI_CMD_IO_CAP_REPLY:
		return len == 9;
	case HCI_CMD_USER_CONFIRM:
	case HCI_CMD_USER_CONFIRM_NEG:
	case HCI_CMD_PASSKEY_NEG:
		return len == BDADDR_LEN;
	case HCI_CMD_SET_EVENT_MASK:
		return len == HCI_EVT_MASK_LEN;
	case HCI_CMD_RESET:
	case HCI_CMD_READ_BUF_SIZE:
		return len == 0;
	case HCI_CMD_WRITE_FLUSH_TO:
		return len == 4;
	case HCI_CMD_WRITE_SSP_MODE:
	case HCI_CMD_WRITE_SECURE_CONNECTIONS:
		return len == 1;
	case HCI_CMD_READ_ENCRYPTION_KEY_SIZE:
		return len == 2;
	default:
		return 0;
	}
}

/*
 * This process is deliberately not pledged: OpenBSD has no pledge promise
 * for USB_DO_REQUEST or the ugen configuration ioctls.  It has no filesystem
 * access beyond inherited descriptors and accepts only the fixed operations
 * validated below.  USB packet parsing and all Bluetooth protocol handling
 * remain in the pledged parent process.
 */
static void
usb_helper_loop(int sock)
{
	struct usb_helper_msg msg;
	struct usb_helper_reply reply;
	struct usb_ctl_request ur;
	unsigned int wlen, wvalue, windex;
	int valid, result;

	close(fd_event);
	for (;;) {
		if (read_full(sock, &msg, sizeof(msg)) == -1)
			break;
		memset(&reply, 0, sizeof(reply));
		result = -1;
		errno = EINVAL;
		if (msg.op == USB_HELPER_REQUEST &&
		    msg.len <= sizeof(msg.data)) {
			ur = msg.ur;
			wlen = UGETW(ur.ucr_request.wLength);
			wvalue = UGETW(ur.ucr_request.wValue);
			windex = UGETW(ur.ucr_request.wIndex);
			valid = (wlen == msg.len);
			if (valid &&
			    ur.ucr_request.bmRequestType == BT_USB_REQTYPE_CMD &&
			    ur.ucr_request.bRequest == BT_USB_REQ_CMD &&
			    msg.len >= HCI_CMD_HDR_LEN &&
			    msg.len <= HCI_CMD_MAX &&
			    msg.data[2] == msg.len - HCI_CMD_HDR_LEN &&
			    usb_hci_command_allowed(get_le16(msg.data),
			    msg.data[2])) {
				ur.ucr_data = msg.data;
				result = ioctl(fd_control, USB_DO_REQUEST, &ur);
			} else if (valid && msg.len == 0 &&
			    ur.ucr_request.bmRequestType == BT_USB_REQTYPE_EP &&
			    ur.ucr_request.bRequest == BT_USB_REQ_CLEAR_FEATURE &&
			    wvalue == BT_USB_FEAT_EP_HALT &&
			    (windex == BT_USB_EP_EVENT ||
			    windex == BT_USB_EP_ACL_IN ||
			    windex == BT_USB_EP_ACL_OUT)) {
				ur.ucr_data = NULL;
				result = ioctl(fd_control, USB_DO_REQUEST, &ur);
			}
		} else if (msg.op == USB_HELPER_BULK_IOCTL) {
			valid = 0;
			if (msg.request == USB_SET_TIMEOUT &&
			    msg.value >= 0 && msg.value <= 60000)
				valid = 1;
			else if (msg.request == USB_SET_SHORT_XFER &&
			    (msg.value == 0 || msg.value == 1))
				valid = 1;
#ifdef USB_SET_BULK_RA
			else if (msg.request == USB_SET_BULK_RA &&
			    (msg.value == 0 || msg.value == 1))
				valid = 1;
#endif
			if (valid)
				result = ioctl(fd_bulk, msg.request, &msg.value);
		}
		if (result == -1)
			reply.error = errno;
		reply.result = result == -1 ? -1 : 0;
		explicit_bzero(&msg, sizeof(msg));
		if (write_full(sock, &reply, sizeof(reply)) == -1)
			break;
		explicit_bzero(&reply, sizeof(reply));
	}
	explicit_bzero(&msg, sizeof(msg));
	explicit_bzero(&reply, sizeof(reply));
	close(fd_control);
	close(fd_bulk);
	close(sock);
	_exit(0);
}

/* Start the broker after firmware setup, when USB endpoints are stable. */
static void
usb_helper_start(void)
{
	int sv[2];
	pid_t pid;

	if (socketpair(AF_UNIX, SOCK_STREAM, PF_UNSPEC, sv) == -1)
		err(1, "socketpair USB helper");
	if (fcntl(sv[0], F_SETFD, FD_CLOEXEC) == -1 ||
	    fcntl(sv[1], F_SETFD, FD_CLOEXEC) == -1)
		err(1, "fcntl USB helper");
	if ((pid = fork()) == -1) {
		close(sv[0]);
		close(sv[1]);
		err(1, "fork USB helper");
	}
	if (pid == 0) {
		close(sv[0]);
		usb_helper_loop(sv[1]);
	}
	close(sv[1]);
	fd_usb_helper = sv[0];
	close(fd_control);
	fd_control = -1;
}

/* Pledge the protocol process after all path and USB setup is done. */
static void
pledge_main(int play_mode)
{
	const char *promises;

	if (play_mode)
		/* sio_open() needs the audio promise even for sndiod devices. */
		promises = "stdio rpath wpath cpath unix tty audio";
	else
		promises = "stdio unix";
	if (pledge(promises, NULL) == -1)
		err(1, "pledge %s", promises);
}

/*
 * Under doas we start as root to open the USB device and, when needed,
 * load firmware.  Normal HCI, key-file and sndio work runs as the user
 * who invoked doas.
 */

static void
drop_privileges(void)
{
	struct passwd *pw;
	const char *user;

	if (geteuid() != 0)
		return;
	if ((user = getenv("DOAS_USER")) == NULL)
		errx(1, "must be run through doas when invoked as root");
	if ((pw = getpwnam(user)) == NULL)
		errx(1, "%s: unknown user", user);
	if (initgroups(pw->pw_name, pw->pw_gid) == -1 ||
	    setresgid(pw->pw_gid, pw->pw_gid, pw->pw_gid) == -1 ||
	    setresuid(pw->pw_uid, pw->pw_uid, pw->pw_uid) == -1)
		err(1, "cannot drop privileges");
	setenv("HOME", pw->pw_dir, 1);
}

static void
onsig(int sig)
{
	(void)sig;
	quit = 1;
}

static __dead void
usage(void)
{
	fprintf(stderr,
	    "usage: %s [-EJSqv] [-f firmware.sfi] [-u unit] "
	    "info | reset | scan | upload\n"
	    "       %s [-JSqv] [-F ms] [-b ms] [-d device] [-n frames] "
	    "[-u unit] play bdaddr\n", getprogname(), getprogname());
	exit(1);
}

/* Parse the command line and run the requested controller operation. */
int
main(int argc, char *argv[])
{
	struct sigaction sa;
	usb_device_descriptor_t dd;
	const char *dev, *fwpath, *errstr;
	int c, unit, ecdsa, nframes, bufms;

	dev = "snd/mon";
	fwpath = NULL;
	unit = 0;
	ecdsa = 0;
	nframes = STREAM_FRAMES_DEFAULT;
	bufms = STREAM_BUFFER_MS;

	while ((c = getopt(argc, argv, "EF:JSqb:d:f:n:u:v")) != -1) {
		switch (c) {
		case 'E':
			ecdsa = 1;
			break;
		case 'F':
			flush_ms = strtonum(optarg, 0, 1000, &errstr);
			if (errstr != NULL)
				errx(1, "-F %s: %s", optarg, errstr);
			break;
		case 'J':
			allow_just_works = 1;
			break;
		case 'S':
			skip_security = 1;
			break;
		case 'b':
			bufms = strtonum(optarg, STREAM_BLOCK_MS * 2, 60000,
			    &errstr);
			if (errstr != NULL)
				errx(1, "-b %s: %s", optarg, errstr);
			break;
		case 'd':
			dev = optarg;
			break;
		case 'f':
			fwpath = optarg;
			break;
		case 'n':
			nframes = strtonum(optarg, 1, MEDIA_MAX_FRAMES,
			    &errstr);
			if (errstr != NULL)
				errx(1, "-n %s: %s", optarg, errstr);
			break;
		case 'u':
			unit = strtonum(optarg, 0, INT_MAX, &errstr);
			if (errstr != NULL)
				errx(1, "-u %s: %s", optarg, errstr);
			break;
		case 'q':
			quiet = 1;
			break;
		case 'v':
			verbose = 1;
			break;
		default:
			usage();
		}
	}
	argc -= optind;
	argv += optind;
	if (argc < 1)
		usage();
	if (skip_security && allow_just_works)
		errx(1, "-J and -S are mutually exclusive");
	if (strcmp(argv[0], "play") == 0 && strncmp(dev, "snd/", 4) != 0)
		errx(1, "unsupported sndio device %s (use snd/)", dev);

	if (allow_just_works)
		qwarnx("-J enabled: Just Works pairing has no MITM protection");
	if (verbose)
		qfprintf(stderr, "btspk %s, built %s %s\n", BTSPK_VERSION,
		    __DATE__, __TIME__);
	umask(077);
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = onsig;		/* no SA_RESTART, reads must stop */
	sigemptyset(&sa.sa_mask);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = SIG_IGN;
	sigemptyset(&sa.sa_mask);
	sigaction(SIGPIPE, &sa, NULL);
	atexit(disconnect);

	usb_open(unit);
	if (ioctl(fd_control, USB_GET_DEVICE_DESC, &dd) == 0 &&
	    UGETW(dd.idVendor) == BT_USB_VENDOR_INTEL) {
		if (strcmp(argv[0], "reset") == 0) {
			intel_reset();
			return 0;
		}
		if (strcmp(argv[0], "upload") == 0) {
			upload(fwpath, ecdsa);
			return 0;
		}
		intel_setup(fwpath, ecdsa, strcmp(argv[0], "info") == 0);
		if (strcmp(argv[0], "info") == 0)
			return 0;
	} else if (strcmp(argv[0], "info") == 0 ||
	    strcmp(argv[0], "reset") == 0 ||
	    strcmp(argv[0], "upload") == 0)
		errx(1, "not an Intel controller");

	/* the priority has to be raised while we are still root */
	if (strcmp(argv[0], "play") == 0)
		setpriority(PRIO_PROCESS, 0, STREAM_NICE);
	drop_privileges();
	if (strcmp(argv[0], "play") == 0) {
		keypath_init();
		sandbox(dev);
	}
	usb_helper_start();
	pledge_main(strcmp(argv[0], "play") == 0);

	hci_init();
	if (strcmp(argv[0], "scan") == 0 && argc == 1)
		scan();
	else if (strcmp(argv[0], "play") == 0 && argc == 2)
		play(argv[1], dev, nframes, bufms);
	else
		usage();
	return 0;
}
