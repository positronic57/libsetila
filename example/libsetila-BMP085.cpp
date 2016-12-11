 /**
  * @file libsetila-BMP085.h
  * @brief An example of using libsetila for temperature and pressure measurement with BMP085/BMP180 sensor.
  *
  * @author Goce Boshkovski
  * @date 01-May-16
  * @copyright GNU General Public License v2.
  *
  */

#include <iostream>
#include <cstdlib>
#include "I2CBus.h"
#include "BMP085.h"
#include "setila_aux.h"
#include "setila_errors.h"

extern "C" {
	#include <getopt.h>
	void print_usage(const char *);
	void parse_opts(int argc, char *argv[]);
}

char I2CAddress;
char *device = (char *) "/dev/i2c-0";
int measureTemperature,measurePressure = 0;
int calculateAltitude = 0;
int addressPresent = 0;
int mode = 1;


int main(int argc, char *argv[])
{

	parse_opts(argc,argv);

	if (!addressPresent)
	{
		std::cout << "Device address is not defined." << std::endl;
		print_usage(argv[0]);
		return -1;
	}

	if (mode<0 || mode>3)
	{
		std::cout << "Unsupported measurement mode " << mode << std::endl;
		print_usage(argv[0]);
		return -1;
	}



	/* The sensor is physically connect to the I2C bus represent with the device name /dev/i2c-1. */
	I2CBus i2cbus(device);

	/* The I2C address of the sensor from Adafruit BMP085 module is 0x77. */
	BMP085 BMP085Sensor(I2CAddress);

	int error_code;

	/* Open the I2C bus device file for communication. */
	if (i2cbus.openI2CBus())
	{
		std::cout << "Cannot connect to I2C bus." << std::endl;
		return ERROR_OPEN_I2C_BUS;
	}

	/* Attach the sensors to the I2C bus */
	BMP085Sensor.attachSensorToI2CBus(i2cbus.getI2CBus());

	/* Init BMP085 sensor*/
	BMP085Sensor.initSensor();


	if (measureTemperature+measurePressure+calculateAltitude)
	{
		std::cout << "BMP085 readings:" << std::endl;
		if (BMP085Sensor.measureTemperaturePressure())
		{
			std::cout << "Failed to retrieve sensor data." << std::endl;
			return -1;
		}
		if (measureTemperature)
			std::cout << "Temperature: t=" << BMP085Sensor.getTemperatureReading() << "[°C]" << std::endl;
		if (measurePressure)
			std::cout << "Pressure: P=" << BMP085Sensor.getPressureReading() << "[hPa]" << std::endl;
		if (calculateAltitude)
			std::cout << "Altitude above the see level: h=" << SetilaAUX::altitude((double)BMP085Sensor.getPressureReading(),(double)AVERAGE_SEA_LEVEL_PRESSURE) << "[m]" << std::endl;
	}

	/* Close the access to the I2C bus. */
	i2cbus.closeI2CBus();

	return 0;
}


void print_usage(const char *prog)
{
        std::cout << "Usage: " << prog << " [-adtTpm]\n" <<
		"  -a --address\t\t mandatory, sets the I2C bus address of the BMP085 sensor;\n" <<
		"  -d --device\t\t set the I2C device (default is /dev/i2c-0);\n" <<
		"  -t --temperature\t measure temperature and print the value in [C];\n" <<
		"  -p --pressure\t\t measure the atmospheric pressure and print the value in [hPa];\n" <<
		"  -A --altitude\t\t calculates the relative amplitude change in [m];\n" <<
		"  -m --mode\t\t sets the measurement mode. Default value 1 = STANDARD. Allowed values:\n" <<
		"								0 = ULTRA LOW POWER\n" <<
		"								1 = STANDARD\n" <<
		"								2 = HIGH RESOLUTION\n" <<
		"								3 = ULTRA HIGH RESOLUTION\n" <<
	std::endl;

	exit(1);
}

void parse_opts(int argc, char *argv[])
{
        while (1) {
        	static const struct option lopts[] = {
                        { "address", required_argument, NULL, 'a' },
                        { "device", required_argument, NULL, 'd' },
			{ "mode", required_argument, NULL, 'm' },
                        { "temperature", no_argument, NULL, 't' },
                        { "pressure", no_argument, NULL, 'p' },
			{ "altitude", no_argument, NULL, 'A' },
                        { NULL, 0, 0, 0 },
        	};

        	int c;

            c = getopt_long(argc, argv, "a:d:m:tpA", lopts, NULL);

            if (c == -1)
            	break;

                switch (c) {
                	case 'd':
                        device = optarg;
                        break;
                	case 'a':
               			{
                        		I2CAddress = (char)strtol(optarg,NULL,0);
                        		addressPresent = 1;
                		}
                		break;
                	case 'm':
                		mode = atoi(optarg);
                		break;
                	case 't':
				measureTemperature = 1;
				break;
                	case 'p':
				measurePressure = 1;
				break;
			case 'A':
				calculateAltitude = 1;
				break;
                	default:
				print_usage(argv[0]);
                        break;
                }
        }
}
