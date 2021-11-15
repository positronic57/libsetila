## Libsetila

Libsetila is a library written in C++ for communication with I2C/SPI slave devices from user space on SOC/SBC running Linux. It provides classes for representing SPI and I2C slave devices and implements data exchange methods that can be used for communication with target devices on I2C/SPI bus. The main purpose of the library is to simplify the process of writing software for communication with different I2C/SPI devices from user space on Linux.

The low level I2C and SPI communication is done by the appropriate Linux kernel drivers while the library use the standard read/write IO calls to send/receive data from/to the slave devices on the bus. The kernel drivers must be loaded before calling the library IO functions.

Main resources providided with the library:
- classes for SPI/I2C master;
- classes for implementing SPI/I2C slaves;
- templates for Basic Data Containers (BDC): stack, FIFO queue and circular buffer. Readings from different SPI/I2C devices can be stored in those data containers; 
- filters module for digital filtering of the sensor readings;
- support for some common/popular SPI/I2C devices (see supported hardware section).
 
The library is not thread safe. 

### Software Requirements

The library requires C++ compiler that supports C++11 standard.

### Supported Hardware

Besides being a general purpose tool for talking with target I2C/SPI devices, it integrates support for some popular devices like: pressure/humidity/temperature sensors, ADC etc.. Those devices were mainly used as test devices during the code development. 

#### Hardware Platforms and Operating System

The library should work on any SOC/SBC with I2C/SPI interface(s), capable of running Linux with appropriate kernel drivers for the master controllers and C++11 compiler.

Successful tests have been done on:
- Raspbery Pi 2, 3 model B and Pi Zero/W all running Rasbian GNU/Linux version 10 ("buster");
- Beagle Bone Black revision A with Debian GNU/Linux 9 ("stretch").

#### I2C Devices

- ST HTS221 humidity and temperature sensor: single acquisition(ONE SHOT measurement), FIFO_MEAN mode, adjustable output data rate and resolution;
- ST LPS25H pressure and temperature sensor: single acquisition(ONE SHOT measurement), adjustable output data rate and resolution;
- ST LPS22HB pressure and temperature sensor: single acquisition(ONE SHOT measurement);
- Microchip MCP9808 temperature sensor: temperature reading, no support for alarms/interrupts;
- Analog Devices ADT7410 temperature sensor: temperature readign, no support for alarms/interrupts;
- Bosh BMP085 pressure and temperature sensor. Fully supported;
- Bosh BMP180 pressure and temperature sensor. Fully supported;
- SRF02 ultra sonic range finder.

#### SPI Devices

- Microchip MCP3204 2.7V, 4-Channel, 12-Bit Analog/Digital Converter

### Installation

For installing instructions check INSTALL file.

### Documentation

Check the doc folder for detailed library documentation.

### Usage and Examples

The library is distributed with examples that demonstrate the library use. Check the code in the devices folder
as an example for communication with different I2C and SPI devices.
The library must be installed before building the examples.

### Test Hardware

The library functionality has been tested using the following hardware:

- Adafruit BMP085 module (Adafruit product ID 391);
- Adafruit BMP180 module (Adafruit product ID 1603);
- Adafruit MCP9808 High Accuracy I2C Temperature Sensor Breakout Board (Adafruit product ID 1782); 
- Adafruit ADT7410 High Accuracy I2C Temperature Sensor Breakout Board (Adafruit product ID 4089);
- [MCP3204 test module](https://github.com/positronic57/libmcp3204/tree/master/example/hardware) 
- SRF02 - I2C/Serial ultrasonic ranger sensor
- Arduino MKR ENV Shield rev2
- Pi Sense HAT;
- Raspbery Pi model B rev1;
- Raspbery Pi 2 model B;
- Raspbery Pi 3 model B;
- Raspbery Pi Zero/W;
- Beagle Bone Black revision A.


**WARNING:** 
The source is provided as it is without any warranty. Use it on your own risk!
The author does not take any responsibility for the damage caused while using this software.

**DISCLAIMER:**
The code is a result of a hobby work and the author is not affiliated with any of the hardware/components/boards manufacturers mentioned in the code, documentation or the description of this project. All trademarks are the property of the respective owners.
