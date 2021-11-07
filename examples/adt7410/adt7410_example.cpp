/**
 * @file ADT7410_example.cpp
 *
 * @brief It demonstrates the use of libsetila library for temperature measurement with ADT7410 sensor.
 * In this example the ADT7410 operates in continuous conversion mode with 16-bit resolution.
 * The temperature reading is done couple of times in a loop.
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 *
 */

#include <iostream>

#include "setila/setila_i2c.h"

#include "setila/ADT7410.h"


int main()
{
	// Example tested on Rasberry Pi rev 1 were I2C interface is /dev/i2c-0. Put the correct device name for your platform
	Bus_Master_Device *i2c_bus_master = new Bus_Master_Device("/dev/i2c-0", BUS_TYPE::I2C_BUS);

	// ADT7410 temperature sensor with the default I2C address of 0x48
	ADT7410 *adt7410_sensor = new ADT7410((uint8_t)ADT7410::I2C_ADDRESS::_0x48);

	int status = 0;

	if (i2c_bus_master->open_bus() < 0)
	{
		std::cout << "Failed to open master bus" << std::endl;
		return -1;
	}

	adt7410_sensor->attach_to_bus(i2c_bus_master);

	// Configure ADT7410 for continuous conversion mode of operation and 16-bit resolution
	status = adt7410_sensor->set_mode_of_operation(ADT7410::MODE_OF_OPERATION::CONTINUOUS_CONVERSION, ADT7410::RESOLUTION::RES_0_0078);
	if(status)
	{
		std::cout << "ADT7410 sensor initialization failed with status " << status << std::endl;
		return status;
	}

	std::cout << std::endl << "ADT7410 temperature readings:" << std::endl << std::endl;

	for(int i = 0; i < 10; i++) {
		// Get the temperature readings
		if ((status = adt7410_sensor->get_sensor_readings()) < 0)
		{
			std::cout << "ADT7410 reading failed with status" << status << "." << std::endl;
			break;
		}
		std::cout << "Temperature T=" << adt7410_sensor->temperature() << "[°C]" << std::endl;
		std::cout << std::endl;
	}

	delete adt7410_sensor;
	delete i2c_bus_master;

	return 0;
}

