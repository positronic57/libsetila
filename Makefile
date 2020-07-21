# Include build definitions for all subfolders in the project folder
-include spi/spi.mk
-include i2c/i2c.mk
-include devices/devices.mk

SUBDIRS := \
. \
devices \
i2c \
spi \

CPP_SRCS += \
../bus_master_device.cpp \
../slave_device.cpp 

OBJS += \
./bus_master_device.o \
./slave_device.o 

SOBJS += \
./stat/bus_master_device.o \
./stat/slave_device.o

CPP_DEPS += \
./bus_master_device.d \
./slave_device.d 

.PHONY: all clean static shared setila stat install uninstall

# Compile the source files in the main folder
%.o: ./%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I$(HEADER_FILES) -O0 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

stat/%.o: ./%.cpp | stat
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I$(HEADER_FILES) -O0 -Wall -c -fmessage-length=0 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

stat:
	@mkdir -p $@

# Define build targets
static: libsetila.a

shared: libsetila.so

all: static shared

.DEFAULT_GOAL := all 

# Build the shared object file
libsetila.so: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++ -shared -o "libsetila.so" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

libsetila.a: $(SOBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	ar qc "libsetila.a" $(SOBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

install: | setila
	@echo ' '
	@echo Installing library files in /usr/local/lib
	@cp libsetila.* /usr/local/lib
	@echo Copy of header files in /usr/local/include/setila
	@cp -r ./include/* /usr/local/include/setila

setila:
	@echo Installing the library requires root privileges
	@echo Create /usr/local/include/setila if not exist
	@mkdir -p /usr/local/include/setila

uninstall:
	@echo Uninstalling the library requires root privileges
	rm /usr/local/lib/libsetila.a
	rm /usr/local/lib/libsetila.so
	rm -r /usr/local/include/setila
	 

# Clean up the files created with build
clean:
	-$(RM) -rf $(OBJS) $(SOBJS) $(CPP_DEPS) libsetila.so libsetila.a stat
	-@echo ' '

