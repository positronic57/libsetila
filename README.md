####Libsetila

Libsetila is a Linux user space library written in C++ for communication with diffrent types of popular sensors.

Currently only I2C sensors are supported. The low level I2C communication is done by the Linux kernel module, while 
the library implements the data exchange protocols and features supported by the sensors. 

###Supported I2C Sensors

Humidity sensors:

- ST HTS221 humidity and temperature sensor.

Pressure sensors:

- ST LPS25H pressure and temperature sensor;
- Bosh BMP085 pressure and temperature sensor;
- Bosh BMP180 pressure and temperature sensor.

###Installation

Installation instruction can be found in the INSTALL file.


###Documentation

Check the doc folder for detailed library documentation.


###Hardware for testing 

The library functionality has been tested using the following hardware:

- Adafruit BMP085 module (Adafruit product ID 391);
- Adafruit BMP180 module (Adafruit product ID 1603);
- Pi Sense HAT;
- Raspbery Pi 1 model B;
- Raspbery Pi 2 model B.  

The library is distributed with examples that demonstrate the library use.


**WARNING:** 
The source is provided as it is without any warranty. Use it on your own risk!
The author does not take any responsibility for the damage caused while using this software.

