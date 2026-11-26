# btspk

`btspk` is an OpenBSD userland Bluetooth A2DP audio client. It talks
 directly to an Intel Bluetooth USB controller through `ugen(4)`, implements
the host-side Bluetooth protocols needed for A2DP, encodes audio with
`libsbc`, and uses `sndio`/`sndiod` for PCM input.

The project is intentionally implemented entirely in userland. OpenBSD does
not provide an in-kernel Bluetooth stack, so `btspk` implements the required
Bluetooth host protocols itself and talks directly to the Bluetooth USB
controller through `ugen(4)`.

## Security model

Security is divided into three layers:

```text
                    btspk
                      |
        +-------------+-------------+
        |                           |
        v                           v
   pledge(2)                    unveil(2)
   syscall policy               filesystem policy
        |                           |
        |                           +-- $HOME / .btspk_keys
        |                           +-- /dev/tty
        |                           +-- /tmp/sndio
        |
        +-------------------+
        |                   |
        v                   v
  pledged main          USB ioctl broker
  process               (small helper)
        |                   |
        | HCI/L2CAP/        | ugen(4) ioctls
        | AVDTP/A2DP        |
        | sndio              v
        |                Intel USB
        |                controller
        v
     Bluetooth
       device
```

The important point is that `pledge(2)` and `unveil(2)` solve different
problems. `pledge()` limits what classes of operations the process may perform,
while `unveil()` limits which filesystem paths remain visible to it.

### Why there is a USB helper

The normal protocol process is pledged, but the USB `ugen(4)` control and
endpoint configuration ioctls used by this program are not covered by a
suitable generic `pledge(2)` promise.

Instead, `btspk` creates a small helper process after USB initialization:

```text
              main process
                   |
                   | fixed-size Unix socketpair
                   | validated requests only
                   v
             USB helper
             /        \
            v          v
       fd_control    fd_bulk
            |          |
            +-----+----+
                  |
                ugen(4)
```

The main process does **not** send arbitrary ioctl numbers to the helper.
Requests are represented by a fixed internal message structure and the helper
accepts only the operations required by the Bluetooth implementation.

For HCI control transfers, the helper additionally checks the USB request type,
request code, transfer length, HCI command length, and HCI opcode. Bulk
configuration requests are restricted to the specific `ugen(4)` operations
used by the program and bounded values.

The helper is deliberately not pledged because it must issue the USB ioctls.
It has no path-based filesystem access of its own; it inherits the descriptors
needed for the USB operation and the IPC socket, and the parent closes its own
USB control descriptor after the helper is started.

## `pledge(2)`

`btspk` uses pledge in stages rather than applying a single broad promise for
the entire lifetime of the program.

### Playback

Before opening the sndio device, the main process uses:

```c
pledge("stdio rpath wpath cpath unix tty audio", NULL);
```

This is the setup phase. The promises cover:

- `stdio` — normal file-descriptor I/O and basic process operations.
- `rpath` — reading the link-key file and other required paths.
- `wpath` — writing the persistent link-key file.
- `cpath` — creating the link-key file when it does not exist.
- `unix` — Unix-domain communication, including the sndiod connection and
  the internal USB-helper socketpair.
- `tty` — the explicit Numeric Comparison confirmation prompt on `/dev/tty`.
- `audio` — required by OpenBSD's `sio_open(3)` path even when the selected
  device is an `snd/...` sndiod device rather than a raw `/dev/audio` device.

After `sio_open()` succeeds, the process narrows its pledge to:

```c
pledge("stdio unix audio", NULL);
```

At that point it no longer needs pathname access for normal streaming and no
longer needs the `tty` promise. The process continues to use `unix` for the
sndiod connection and the `audio` promise for the sndio API.

### Scan and non-playback operation

For operations that do not open an sndio playback device, the main process
uses:

```c
pledge("stdio unix", NULL);
```

This is intentionally smaller than the playback setup promise.

### What pledge protects

`pledge()` is not a Bluetooth authentication mechanism and does not make the
USB controller trustworthy. It is a post-setup syscall/operation sandbox.

For example, after the playback pledge is narrowed, code that unexpectedly
tries to perform a filesystem pathname operation is outside the process's
allowed promise set and is terminated by the kernel rather than being allowed
to continue normally.

The security boundary is therefore:

```text
   application bug / compromised parser
                  |
                  v
        +-------------------+
        | pledged main      |
        | process            |
        +-------------------+
          |       |       |
          |       |       +-- no pathname operations after setup
          |       +---------- no tty access after setup
          +------------------ only promised operation classes
```

The USB helper is a separate boundary. It is intentionally small because
`pledge()` cannot be used to authorize the required USB ioctls in the main
process.

## `unveil(2)`

`unveil()` provides the filesystem half of the sandbox. For playback,
`btspk` resolves the user's home directory and key-file path before locking
down filesystem visibility.

The current playback sandbox is:

```text
unveil($HOME,     "rwc")
unveil(/dev/tty,   "rw")
unveil(/tmp/sndio, "w")
unveil(NULL,      NULL)       /* lock the policy */
```

The meaning is:

```text
$HOME
  |
  +-- ~/.btspk_keys
       read existing keys
       create/update private key file

/dev/tty
  |
  +-- Numeric Comparison confirmation

/tmp/sndio
  |
  +-- sndiod Unix-domain socket

Everything else
  |
  +-- hidden by unveil(2) after the policy is locked
```

The `/tmp/sndio` rule is important. `btspk` is an **sndiod client**, not a
listener. When `sio_open("snd/mon", ...)` is used, sndio connects to the local
sndiod Unix socket. The working OpenBSD configuration uses `/tmp/sndio` for
that socket.

`btspk` does not require direct `/dev/audio` access for the normal `snd/...`
playback path.

## Privilege separation and setup order

The program may start with elevated privileges when invoked through `doas` so
that it can open the USB controller and perform controller/firmware operations.
For normal playback, it then drops to the invoking user before ordinary HCI,
key-file, and sndio work continues.

The important ordering is:

```text
 open USB controller
        |
        v
 Intel controller setup / firmware
        |
        v
 raise scheduling priority if requested
        |
        v
 drop privileges to DOAS_USER
        |
        +--> resolve ~/.btspk_keys
        |
        +--> unveil filesystem paths
        |
        v
 start USB ioctl helper
        |
        v
 pledge main process
        |
        v
 HCI / L2CAP / AVDTP / A2DP
        |
        v
 sndio playback
```

Resolving the user's home directory before `unveil()` is intentional: once the
filesystem policy is locked, the password database and unrelated filesystem
paths are no longer available to the main process.

## Pairing security

The default pairing policy requires authenticated Secure Simple Pairing using
Numeric Comparison. The user must explicitly confirm the displayed value.

If the remote device cannot provide compatible Numeric Comparison
capabilities, normal pairing is refused.

### Just Works

Some devices cannot perform Numeric Comparison. `-J` explicitly opts into
Just Works pairing:

```sh
doas ./btspk -J -n 2 play 04:fe:a1:8c:3a:fb
```

Just Works does **not** provide MITM protection. `btspk` warns when `-J` is
enabled. This option is intended for devices where authenticated Numeric
Comparison is unavailable and the user accepts the weaker pairing security.

`-J` and `-S` are mutually exclusive.

### Diagnostic security bypass

`-S` disables the normal authentication/encryption handling and is intended
only for diagnostics. It should not be used for normal connections or with
untrusted devices.

```sh
doas ./btspk -S -v -n 2 play 04:fe:a1:8c:3a:fb
```

## Link-key storage

Authenticated link keys are stored in:

```text
~/.btspk_keys
```

The file is created with a restrictive umask and is accepted only when it is a
regular file owned by the current user with no group/other permission bits.
The file size is also bounded.

Sensitive key material and temporary HCI buffers are explicitly cleared where
appropriate with `explicit_bzero()`.

## Requirements

- OpenBSD.
- An Intel Bluetooth USB controller supported by the firmware and USB
  transport code. The example below uses an Intel AX201.
- `libsbc`.
- `sndio` / `sndiod`.
- Permission to access the relevant `ugen(4)` device nodes.

`btspk` communicates with the USB controller directly through OpenBSD's USB
interfaces. It does not use `libusb`.

## Install `libsbc`

Install the port under `mystuff`:

```sh
cd /usr/ports/mystuff/audio/libsbc
make install clean
```

## Intel firmware

The Intel AX201 example uses:

```text
ibt-19-0-4.sfi
```

Inspect the controller:

```sh
doas ./btspk info
```

Upload firmware as a standalone operation:

```sh
doas ./btspk -f ibt-19-0-4.sfi upload
```

Reset the controller into its bootloader state with:

```sh
doas ./btspk reset
```

After reset or firmware upload, the controller may re-enumerate under a
different `ugen` unit. Check `dmesg` and use `-u` when necessary.

For firmware images using the ECDSA header format, use `-E`:

```sh
doas ./btspk -E -f firmware.sfi upload
```

## Configure `sndiod`

The default playback input is `snd/mon`.

A working monitor configuration is:

```sh
doas rcctl set sndiod flags -s default -m mon -s mon
doas rcctl restart sndiod
```

Inspect the available sndio devices with:

```sh
sndioctl -d
```

The sandboxed sndiod connection uses the local Unix socket directory:

```text
/tmp/sndio
```

## Usage

```text
btspk [-EJSqv] [-f firmware.sfi] [-u unit] info | reset | scan | upload
btspk [-JSqv] [-F ms] [-b ms] [-d device] [-n frames] [-u unit] play bdaddr
```

### Controller information

```sh
doas ./btspk info
```

### Scan

```sh
doas ./btspk scan
```

### Play

```sh
doas ./btspk -n 2 play 04:fe:a1:8c:3a:fb
```

Select a different `sndio` device with `-d`:

```sh
doas ./btspk -d snd/mon -n 2 play 04:fe:a1:8c:3a:fb
```

The default SBC packetization is two frames per RTP media packet and the
default sndio application buffer is 30 ms. Playback continues until
interrupted with Ctrl-C.

### Quiet mode

`-q` suppresses informational and non-fatal warning output while leaving fatal
errors visible.

```sh
doas ./btspk -q -n 2 play 04:fe:a1:8c:3a:fb
```

The Numeric Comparison confirmation prompt is still shown when required,
because silently suppressing that prompt would defeat the explicit security
confirmation.

## Audio pipeline

```text
 sndio / sndiod
       |
       | PCM
       v
   btspk audio path
       |
       | SBC
       v
    libsbc
       |
       | RTP media packets
       v
 AVDTP / L2CAP
       |
       v
 Bluetooth ACL
       |
       v
   A2DP device
```

The negotiated A2DP sample rate and SBC configuration determine the encoder
parameters. `-n` controls the number of SBC frames placed in each media packet
and `-b` controls the sndio application buffer size in milliseconds.

## Filesystem and syscall sandbox summary

| Component | Access |
|---|---|
| Main playback process | `pledge()` after setup; `unveil()` locked |
| Link-key file | `$HOME/.btspk_keys` |
| Numeric Comparison | `/dev/tty` during setup/pairing |
| sndio/sndiod | `/tmp/sndio` Unix socket |
| Raw `/dev/audio` | Not required for `snd/...` playback |
| USB controller | Isolated through the USB helper |
| USB helper | Unpledged, fixed IPC protocol, bounded USB operations |

The design deliberately keeps the Bluetooth protocol implementation in the
pledged main process while moving only the unavoidable `ugen(4)` ioctl surface
into the small helper.

## Project status

This is a native OpenBSD userland Bluetooth A2DP implementation. The source
is closely tied to OpenBSD interfaces such as `ugen(4)`, `pledge(2)`,
`unveil(2)`, and `sndio(7)`.
