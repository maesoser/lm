CC = gcc
CFLAGS += -O2 -Wall
PREFIX ?= /usr/local

all: o4l

o4l:
	$(CC) $(CFLAGS) *.c -o o4l

install: o4l
	install -d $(PREFIX)/bin
	install -m755 o4l $(PREFIX)/bin/o4l

clean:
	rm -f o4l
