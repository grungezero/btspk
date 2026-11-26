# $OpenBSD$

COMMENT = Bluetooth low-complexity subband codec library

V =2.2
DISTNAME =sbc-$V
CATEGORIES =audio

HOMEPAGE =https://www.bluez.org/

MAINTAINER =	Francois Briere <fbriere@ik.me>

# GPLv2+ (library components also carry LGPL-2.1-or-later notices)
PERMIT_PACKAGE =Yes

WANTLIB += c m

SITES =https://www.kernel.org/pub/linux/bluetooth/

USE_GMAKE=Yes

CONFIGURE_STYLE =gnu
CONFIGURE_ARGS =--disable-tools \
		--disable-tester \
		--disable-static

# Confirm the ABI version with the generated shared_libs.log after first build.
SHARED_LIBS +=  sbc                      4.2# 4.2
.include <bsd.port.mk>
