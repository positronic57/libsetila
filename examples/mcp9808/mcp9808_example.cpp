/**
 * @file mcp9808.cpp
 * @brief Reads temperature from MCP9808 sensor using libsetila library.
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 *
 */

#include <iostream>

#include "setila/setila_i2c.h"

#include "setila/mcp9808.h"


int main()
{
	// Example tested on Rasberry Pi rev 1 were I2C interface is /dev/i2c-0. Put the correct device name for your platform
	Bus_Master_Device *i2c_bus_master = new Bus_Master_Device("/dev/i2c-0", BUS_TYPE::I2C_BUS);

	// MCP9808 temperature sensor with the defaule I2C address of 0x18
	MCP9808 *mcp9808_sensor = new MCP9808(MCP9808_I2C_ADDR);

	int status = 0;

	if (i2c_bus_master->open_bus() < 0)
	{
		std::cout << "Failed to open master bus" << std::endl;
		return -1;
	}

	mcp9808_sensor->attach_to_bus(i2c_bus_master);
	// Configure MCP9808 for continuous conversion mode of operation
	status = mcp9808_sensor->set_mod_of_operation(MCP9808::MODE_OF_OPERATION::CONTINUOUS_CONVERSION, MCP9808::RESOLUTION::RES_0_5_DEG);
	if(status)
	{
		std::cout << "MCP9808 sensor initialization failed with status " << status << std::endl;
		return status;
	}

	// Get the temperature readings
	if (mcp9808_sensor->get_sensor_readings())
	{
		std::cout << "MCP9808 reading failed." << std::endl;
		return -1;
	}

	std::cout << std::endl << "Ambient temperature reading:" << std::endl << std::endl;
	std::cout << "Temperature T=" << mcp9808_sensor->ambient_temperature() << "[°C]" << std::endl;
	std::cout << std::endl;

	delete mcp9808_sensor;
	delete i2c_bus_master;

	return 0;
}

