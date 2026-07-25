</$objtype/mkfile

LIB=libview.a$O

OFILES=\
	v3.$O\
	orbit.$O\
	spring.$O\

HFILES=\
	view.h\

TARG=viewtest

default:V: all

all:V: $LIB $TARG

$LIB: $OFILES
	ar rvc $LIB $OFILES

viewtest: viewtest.$O $LIB
	$LD -o $target viewtest.$O $LIB

%.$O: %.c $HFILES
	$CC $CFLAGS $stem.c

LIBDIR=$home/lib
INCDIR=$home/include

install:V: $LIB
	mkdir -p $LIBDIR $INCDIR
	cp $LIB $LIBDIR/$LIB
	cp view.h $INCDIR/view.h

clean:V:
	rm -f *.[$OS] [$OS].out $LIB $TARG
