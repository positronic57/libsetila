Libsetila
==========

Libsetila is a Linux user space library written in C for communication with sensors present on Raspberry Pi Sense HAT.
It provides access to sensors on Sense HAT using I2C kernel module present on the system.
The low level I2C communication is done by the kernel module, while the library implements the data exchange protocols
supported by the sensors on Sense HAT.

Supported Sensors
-----------------

Current version of the library implements support for HTS221 humidity sensor.
It can read the temperature and humidity from HTS221.


Installation
---------------

Installation instruction can be found in the INSTALL file.


Documentation
---------------
Check the doc folder for detailed library documentation.


Tested on Raspberry Pi 2 with Sense HAT.

**WARNING:** 
The source is provided as it is without any warranty. Use it on your own risk!
The author does not take any responsibility for the damage caused while using this software.

