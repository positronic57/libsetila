# Makefile for building the examples distributed 
# with libsetila.

# List of source files
SOURCES += pi_sense_hat.cpp

#List of object files
OBJECTS += $(SOURCES:.cpp=.o)

#Compile all of the source files
%.o: %.cpp
	@echo Compiling file: $<
	g++ -std=c++0x -O0 -Wall -c -fmessage-length=0 "$^" -o "$@"
	@echo

#Building the binary
pi_sense_hat: pi_sense_hat.o
	@echo Building target file: $@.
	g++ -o $@ $^ -Wl,-Bdynamic -lsetila -lm
	@echo

.PHONY: clean

clean:
	rm -f *.o pi_sense_hat

