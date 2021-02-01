CC=arm-linux-gnueabihf-gcc

CFLAGS=-g
LDFLAGS=-lpthread -lsqlite3

OBJS=app.o

all: app

app: ${OBJS}
	${CC} ${CFLAGS} $? -o $@ ${LDFLAGS}

.PHONY: clean

clean:
	rm -f *.o
	rm -f app

