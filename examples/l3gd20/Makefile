# Makefile for building the examples for ST L3GD20 sensor distributed 
# with libsetila.

all: l3gd20_single_meas_example l3gd20_FIFO_example

.PHONY: clean all

# List of source files
SOURCES += \
l3gd20_single_meas_example.cpp \
l3gd20_FIFO_example.cpp

#List of object files
OBJECTS += $(SOURCES:.cpp=.o)

#Compile all of the source files
%.o: %.cpp
	@echo Compiling file: $<
	g++ -std=c++17 -O0 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo

#Building the binary
l3gd20_single_meas_example: l3gd20_single_meas_example.o
	@echo Building target file: $@.
	g++ -o $@ $^ -Wl,-Bdynamic -lsetila -lm
	@echo

#Building the binary
l3gd20_FIFO_example: l3gd20_FIFO_example.o
	@echo Building target file: $@.
	g++ -o $@ $^ -Wl,-Bdynamic -lsetila -lm
	@echo

clean:
	rm -f *.o *.d l3gd20_single_meas_example l3gd20_FIFO_example

