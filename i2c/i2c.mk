HEADER_FILES = $(CURDIR)/include

CPP_SRCS += \
./i2c/i2c_bus.cpp \
./i2c/i2c_slave_device.cpp 

OBJS += \
./i2c/i2c_bus.o \
./i2c/i2c_slave_device.o 

SOBJS += \
./i2c/stat/i2c_bus.o \
./i2c/stat/i2c_slave_device.o


CPP_DEPS += \
./i2c/i2c_bus.d \
./i2c/i2c_slave_device.d 


# Each subdirectory must supply rules for building sources it contributes
i2c/%.o: ./i2c/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I$(HEADER_FILES) -O0 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

i2c/stat/%.o: ./i2c/%.cpp | i2c/stat
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I$(HEADER_FILES) -O0 -Wall -c -fmessage-length=0 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

i2c/stat:
	@mkdir -p $@
