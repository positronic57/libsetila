# Makefile for building the examples distributed 
# with libsetila.

# List of source files
SOURCES += arduino_mkr_env_shield_rev2.cpp

#List of object files
OBJECTS += $(SOURCES:.cpp=.o)

#Compile all of the source files
%.o: %.cpp
	@echo Compiling file: $<
	g++ -std=c++0x -O0 -Wall -c -fmessage-length=0 "$^" -o "$@"
	@echo

#Building the binary
arduino_mkr_env_shield_rev2: arduino_mkr_env_shield_rev2.o
	@echo Building target file: $@.
	g++ -o $@ $^ -Wl,-Bdynamic -lsetila -lm
	@echo

.PHONY: clean

clean:
	rm -f *.o arduino_mkr_env_shield_rev2

