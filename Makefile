############################################################################
# Makefile for building and installing libsetila
############################################################################

RM := rm -rf

PREFIX = /usr

CC = gcc
CFLAGS = -O3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP
LDFLAGS = -shared
SOURCES = libsetila.c
OBJECTS = $(SOURCES:.c=.o)
	
all: libsetila.so

libsetila.so: $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

libsetila.o: $(SOURCES)
	$(CC) $(CFLAGS) $< -o $@

clean:
	-$(RM) libsetila.o libsetila.so libsetila.d

# Install the library in $(prefix)
install: 
	install -m 0644 libsetila.so $(PREFIX)/lib
	install -m 0644 setila.h $(PREFIX)/include

# Uninstall the library 
uninstall:
	-$(RM) $(PREFIX)/lib/libsetila.so $(PREFIX)/include/libsetila.h


