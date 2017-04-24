CC = gcc
CFLAGS += -Wall
PREFIX ?= /usr/local

all: lm

lm:
	$(CC) $(CFLAGS) *.c -o lm

install: o4l
	install -d $(PREFIX)/bin
	install -m755 lm $(PREFIX)/bin/lm

clean:
	rm -f lm
