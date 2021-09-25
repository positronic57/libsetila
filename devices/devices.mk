HEADER_FILES = $(CURDIR)/include

CPP_SRCS += \
../devices/BMP085.cpp \
../devices/HTS221.cpp \
../devices/LPS22HB.cpp \
../devices/LPS25H.cpp \
../devices/mcp3204.cpp \
../devices/srf02.cpp \
../devices/ST_Sensor.cpp

OBJS += \
./devices/BMP085.o \
./devices/HTS221.o \
./devices/LPS22HB.o \
./devices/LPS25H.o \
./devices/mcp3204.o \
./devices/srf02.o \
./devices/ST_Sensor.o

CPP_DEPS += \
./devices/BMP085.d \
./devices/HTS221.d \
./devices/LPS22HB.d \
./devices/LPS25H.d \
./devices/mcp3204.d \
./devices/srf02.d \
./devices/ST_Sensor.d

SOBJS += \
./devices/stat/BMP085.o \
./devices/stat/HTS221.o \
./devices/stat/LPS22HB.o \
./devices/stat/LPS25H.o \
./devices/stat/mcp3204.o \
./devices/stat/srf02.o \
./devices/stat/ST_Sensor.o



# Each subdirectory must supply rules for building sources it contributes
devices/%.o: ./devices/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I$(HEADER_FILES) -O0 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

devices/stat/%.o: ./devices/%.cpp | devices/stat
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I$(HEADER_FILES) -O0 -Wall -c -fmessage-length=0 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

devices/stat:
	@mkdir -p $@
