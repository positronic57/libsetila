/**
 * @file pi_sense_hat.cpp
 *
 * @brief Get temperature, pressure and humidity readings from Raspberry Pi Sense HAT LPS25H and HTS221 sensors
 * using libsetila library.
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 *
 */

#include <iostream>

#include "setila/setila_i2c.h"
#include "setila/LPS25H.h"
#include "setila/HTS221.h"

int main()
{
	
	Bus_Master_Device *i2c_bus_master = new Bus_Master_Device("/dev/i2c-1", BUS_TYPE::I2C_BUS);

	LPS25H *lps25h_sensor = new LPS25H(0x5C);
	HTS221 *hts221_sensor = new HTS221(0x5F);

	int status;

	if (i2c_bus_master->open_bus() < 0)
	{
		std::cout << "Failed to open master bus" << std::endl;
		return -1;
	}

	status = lps25h_sensor->attach_to_bus(i2c_bus_master);
	status = hts221_sensor->attach_to_bus(i2c_bus_master);

	status = lps25h_sensor->init_sensor();

	if(status)
	{
		std::cout << "LPS25H sensor initialization failed." << std::endl;
		return status;
	}

	status = hts221_sensor->init_sensor(0x1B, 0x85);
	if (status)
	{
		std::cout << "HTS221 sensor initialization failed." << std::endl;
		return status;
	}

	/* Get the humidity and temperature readings from the sensor and calculate the current temperature and humidity values. */
	if (hts221_sensor->get_sensor_readings())
	{
		std::cout << "HTS221 humidity/temperature measurement failed." << std::endl;
		return -1;
	}


	if (lps25h_sensor->get_sensor_readings())
	{
		std::cout << "LPS25H pressure/temperature measurement failed." << std::endl;
		return status;
	}

	std::cout << std::endl << "Readings from Pi Sense Hat sensors:" << std::endl << std::endl;
	std::cout << "LPS25H:" << std::endl;
	std::cout << "Pressure P=" << lps25h_sensor->pressure_reading() << "[hPa]" << std::endl;
	std::cout << "Temperature T=" << lps25h_sensor->temperature_reading() << "[°C]" << std::endl;
	std::cout << std::endl;

	std::cout << std::endl;
	std::cout << "HTS221:" << std::endl;
 	std::cout << "Relative Humidity R=" << hts221_sensor->humidity_reading() << "[%rH]" << std::endl;
	std::cout << "Temperature T=" << hts221_sensor->temperature_reading() << "[°C]" << std::endl;
	std::cout << std::endl;

	delete lps25h_sensor;
	delete hts221_sensor;
	delete i2c_bus_master;

	return 0;
}

