# Makefile for building the examples distributed 
# with libsetila.

# Header files path (Update this with the library installation folder).
HEADERS_PATH = /usr/include

# Library path (Update this with the library installation folder).
LIBRARY_PATH = /usr/lib

# List of source files
SOURCES += libsetila_example.c

#List of object files
OBJECTS += $(SOURCES:.c=.o)

#Compile all of the source files
%.o: %.c
	@echo Compiling file: $<
	gcc -c $^ -o $@
	@echo

#Building libMAX7221example executable
libsetila_example: libsetila_example.o
	@echo Building target file: $@. 
	gcc -o $@ $^ -L$(LIBRARY_PATH) -lsetila
	@echo 

.PHONY: clean

clean:
	rm -f *.o libsetila_example
