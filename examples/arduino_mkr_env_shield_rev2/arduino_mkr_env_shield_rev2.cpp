/**
 * @file arduino_mkr_env_shield_rev2.cpp
 *
 * @brief Get temperature, pressure and humidity readings from Arduino MKR ENV Shield Rev2
 * LPS22HB and HTS221 sensors using libsetila library. 
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 *
 */

#include <iostream>

#include "setila/setila_i2c.h"
#include "setila/LPS22HB.h"
#include "setila/HTS221.h"

int main()
{
	float pressure = 0.0;
	float temperature = 0.0;

	Bus_Master_Device *i2c_bus_master = new Bus_Master_Device("/dev/i2c-0", BUS_TYPE::I2C_BUS);

	// Pressure and temperature sensor
	LPS22HB *lps22hb_sensor = new LPS22HB(LPS22HB_ADDR_SA0_0);  //I2C addess 0x5C

	// Humidity and temperature sensor
	HTS221 *hts221_sensor = new HTS221(0x5F);

	int status = 0;

	if (i2c_bus_master->open_bus() < 0)
	{
		std::cout << "Failed to open master bus" << std::endl;
		return -1;
	}

	status = lps22hb_sensor->attach_to_bus(i2c_bus_master);
	status = hts221_sensor->attach_to_bus(i2c_bus_master);

	// Configure LPS22HB for ONE SHOT mode of operation
	status = lps22hb_sensor->init_sensor();
	if(status)
	{
		std::cout << "LPS22HB sensor initialization failed." << std::endl;
		return status;
	}

	status = hts221_sensor->init_sensor(0x1B, 0x85);
	if (status)
	{
		std::cout << "HTS221 sensor initialization failed." << std::endl;
		return status;
	}

	// Measure pressure/temperature and get the readings using ONE SHOT command
	if (lps22hb_sensor->do_one_shot_measurement(pressure, temperature))
	{
		std::cout << "LPS22HB pressure/temperature measurement failed." << std::endl;
		return status;
	}
	// Measure humidity/temperature and get the readings
	if (hts221_sensor->get_sensor_readings())
	{
		std::cout << "HTS221 humidity/temperature measurement failed." << std::endl;
		return -1;
	}

	std::cout << std::endl << "Readings from Arduino MKR ENV Shield Rev2:" << std::endl << std::endl;
	std::cout << "LPS22HB sensor:" << std::endl;
	std::cout << "Pressure P=" << lps22hb_sensor->last_pressure_reading() << "[hPa]" << std::endl;
	std::cout << "Temperature T=" << lps22hb_sensor->last_temperature_reading() << "[°C]" << std::endl;
	std::cout << std::endl;

	std::cout << "HTS221 sensor:" << std::endl;
	std::cout << "Relative Humidity R=" << hts221_sensor->humidity_reading() << "[%rH]" << std::endl;
	std::cout << "Temperature T=" << hts221_sensor->temperature_reading() << "[°C]" << std::endl;
	std::cout << std::endl;

	delete lps22hb_sensor;
	delete hts221_sensor;
	delete i2c_bus_master;

	return 0;
}

