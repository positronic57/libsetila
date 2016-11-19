#Makefile that builds libsetila library.

#Project name
LIBRARY_NAME = libsetila

# Prefix
PREFIX = /usr

# List of source files
SOURCES += I2CBus.cpp I2CSensor.cpp LPS25H.cpp HTS221.cpp BMP085.cpp setila_aux.cpp 

# List of object files
OBJECTS += $(SOURCES:.cpp=.o)

# Headers 
HEADERS += $(SOURCES:.cpp=.h) setila_aux.h setila_errors.h

# Define C++ compiler flags
override CXXFLAGS = -O3 -Wall -fmessage-length=0 -std=c++0x -fPIC

# Linker flags
LDFLAGS = -shared

all: $(LIBRARY_NAME).so

# Compile all of the source files
%.o: %.cpp
	@echo Compiling file: $<
	g++ $(CXXFLAGS) -c $< -o $@ 
	@echo

# Build the binary
$(LIBRARY_NAME).so: $(OBJECTS)
	@echo Building target file: $@. 
	g++ $(LDFLAGS) -o $@ $^ -lm


# Install the library in $(prefix)
install: 
	install -m 0644 $(LIBRARY_NAME).so $(PREFIX)/lib
	install -m 0644 I2CBus.h $(PREFIX)/include
	install -m 0644 I2CSensor.h $(PREFIX)/include
	install -m 0644 LPS25H.h $(PREFIX)/include
	install -m 0644 HTS221.h $(PREFIX)/include
	install -m 0644 BMP085.h $(PREFIX)/include
	install -m 0644 setila_aux.h $(PREFIX)/include
	install -m 0644 setila_errors.h $(PREFIX)/include

# Uninstall the library from the system
uninstall:
	rm $(PREFIX)/lib/$(LIBRARY_NAME).so
	install -m 0644 I2CBus.h $(PREFIX)/include
	install -m 0644 I2CSensor.h $(PREFIX)/include
	install -m 0644 LPS25H.h $(PREFIX)/include
	install -m 0644 HTS221.h $(PREFIX)/include
	install -m 0644 BMP085.h $(PREFIX)/include
	install -m 0644 setila_aux.h $(PREFIX)/include
	install -m 0644 setila_errors.h $(PREFIX)/include

.PHONY: clean
clean:
	rm $(LIBRARY_NAME).so $(OBJECTS)
