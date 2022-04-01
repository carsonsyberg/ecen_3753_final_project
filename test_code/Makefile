UNAME=$(shell uname)

CCFLAGS=-Wall -O3 -std=gnu99
CC=gcc

all: final

final: main.o final.o final.h finaltests.o
	$(CC) $(LDFLAGS) main.o final.o finaltests.o -o final -lm

remake: clean all

%.o: %.c ctest.h
	$(CC) $(CCFLAGS) -c -o $@ $<

clean:
	rm -f final *.o
