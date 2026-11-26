PROG=	btspk
MAN=	btspk.1

PREFIX?=	/usr/local
BINDIR=	${PREFIX}/bin
MANDIR=	${PREFIX}/man/man1

CFLAGS+=	-Wall -pthread -I/usr/local/include
LDFLAGS+=	-pthread -L/usr/local/lib
LDADD+=	-lsndio -lsbc

.include <bsd.prog.mk>
