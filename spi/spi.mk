HEADER_FILES = $(CURDIR)/include

CPP_SRCS += \
./spi/spi_bus.cpp \
./spi/spi_bus_master.cpp \
./spi/spi_slave_device.cpp 

OBJS += \
./spi/spi_bus.o \
./spi/spi_bus_master.o \
./spi/spi_slave_device.o 

SOBJS += \
./spi/stat/spi_bus.o \
./spi/stat/spi_bus_master.o \
./spi/stat/spi_slave_device.o

CPP_DEPS += \
./spi/spi_bus.d \
./spi/spi_bus_master.d \
./spi/spi_slave_device.d 


# Each subdirectory must supply rules for building sources it contributes
spi/%.o: ./spi/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I$(HEADER_FILES) -O0 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

spi/stat/%.o: ./spi/%.cpp | spi/stat
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I$(HEADER_FILES) -O0 -Wall -c -fmessage-length=0 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

spi/stat:
	@mkdir -p $@
