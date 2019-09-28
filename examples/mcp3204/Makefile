# Makefile for building the examples distributed 
# with libsetila2.

# Project name
PROJECT = mcp3204_example

# List of source files
SOURCES += mcp3204_example.cpp

#List of object files
OBJECTS += $(SOURCES:.cpp=.o)

all: $(PROJECT) 

#Compile all of the source files
%.o: %.cpp
	@echo Compiling file: $<
	g++ -std=c++0x -O0 -Wall -c -fmessage-length=0 "$^" -o "$@"
	@echo

#Building the binary
$(PROJECT): $(OBJECTS)
	@echo Building target file: $@.
	g++ -o $@ $^ -Wl,-Bdynamic -lsetila -lm
	@echo

.PHONY: clean

clean:
	rm -f *.o $(PROJECT) 

