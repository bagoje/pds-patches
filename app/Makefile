CC=arm-linux-gnueabihf-gcc
MCPU=cortex-a9

CFLAGS=-g
LDFLAGS=-lpthread 

OBJS=app.o

all: app

app: ${OBJS}
	${CC} ${CFLAGS} $? -o $@ ${LDFLAGS}

.PHONY: clean

clean:
	rm -f *.o
	rm -f app

