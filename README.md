## Libsetila

Libsetila is a library written in C++ for communication with I2C/SPI slave devices from Linux user space. It provides classes for representing SPI and I2C slave devices and implements data exchange methods that can be used for communication with target devices on I2C/SPI bus. The main purpose of the library is to simplify the process of writing software for communication with different I2C/SPI devices from user space on Linux.

The low level I2C and SPI communication is done by the appropriate Linux kernel drivers while the library use the standard read/write IO calls to send/receive data from/to the slave devices on the bus. The kernel drivers must be loaded before calling the library IO functions.

The library is not thread safe. 

### Software Requirements

The library requires C++ compiler that supports C++11 standard.

### Supported Hardware

Besides being a general purpose tool for talking with target I2C/SPI devices, it integrates support for some popular devices like: pressure/humidity/temperature sensors, ADC etc.. Those devices were mainly used as test devices during the code development. 

#### I2C Devices

- ST HTS221 humidity and temperature sensor: single acquisition(ONE SHOT measurement), FIFO_MEAN mode, adjustable output data rate and resolution.
- ST LPS25H pressure and temperature sensor: single acquisition(ONE SHOT measurement), adjustable output data rate and resolution.
- Bosh BMP085 pressure and temperature sensor. Fully supported.
- Bosh BMP180 pressure and temperature sensor. Fully supported.
- SRF02 ultra sonic range finder

#### SPI Devices

- Microchip MCP3204 Analog Digital Converter

#### Hardware Platforms

Libsetila is tested on different models and generations of Raspbery Pi all running Rasbian GNU/Linux version 10 ("buster"). The library should work on any other Linux system with: I2C/SPI master controller, appropriate kernel drivers for the master controllers and C++11 compiler.

###Installation

For installing instructions check INSTALL file.

### Documentation

Check the doc folder for detailed library documentation.

### Usage and Examples

The library is distributed with examples that demonstrate the library use. Check the code in the devices folder
as an example for communication with different I2C and SPI devices.

### Hardware for testing 

The library functionality has been tested using the following hardware:

- Adafruit BMP085 module (Adafruit product ID 391);
- Adafruit BMP180 module (Adafruit product ID 1603);
- [MCP3204 test module](https://github.com/positronic57/libmcp3204/tree/master/example/hardware) 
- SRF02 - I2C/Serial ultrasonic ranger sensor
- Pi Sense HAT;
- Raspbery Pi 2 model B;
- Raspbery Pi 3 model B;
- Raspbery Pi Zero/W.


**WARNING:** 
The source is provided as it is without any warranty. Use it on your own risk!
The author does not take any responsibility for the damage caused while using this software.

