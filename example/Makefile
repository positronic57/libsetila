# Makefile for building the examples distributed 
# with libsetila.

# Header files path (Update this with the library installation folder).
HEADERS_PATH = /usr/include

# Library path (Update this with the library installation folder).
LIBRARY_PATH = /usr/lib

# List of source files
SOURCES += libsetila-BMP085.cpp libsetila-SenseHAT.cpp

#List of object files
OBJECTS += $(SOURCES:.cpp=.o)

all: libsetila-BMP085 libsetila-SenseHAT

#Compile all of the source files
%.o: %.cpp
	@echo Compiling file: $<
	g++ -I $(HEADERS_PATH) -std=c++0x -c $^ -o $@
	@echo

#Building libsetila-BMP085 executable
libsetila-BMP085: libsetila-BMP085.o
	@echo Building target file: $@. 
	g++ -L$(LIBRARY_PATH) -o $@ $^ -lsetila 
	@echo 

#Building libsetila-SenseHAT executable
libsetila-SenseHAT: libsetila-SenseHAT.o
	@echo Building target file: $@. 
	g++ -L$(LIBRARY_PATH) -o $@ $^ -lsetila 
	@echo 


.PHONY: clean

clean:
	rm -f *.o *.d libsetila-BMP085 libsetila-SeseHAT

