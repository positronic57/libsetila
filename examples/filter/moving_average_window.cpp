/**
 * @file moving_average_window.cpp
 *
 * @brief Example of using libsetila filter module for filtering the pressure reading from BMP085 pressure sensor.
 * Each pressure sample will pass through a simple moving average window filter with window size of 5 samples.
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 */

#include <iostream>
#include <thread>         
#include <chrono>         

#include "setila/setila_i2c.h"
#include "setila/BMP085.h"
#include "setila/filters/Moving_Average_Filter.h"

int main(void)
{
	//The sensor is connected to I2C master represented with i2c-0 device. Put the correct device name for your platform (can be: i2c-1 etc..)
	Bus_Master_Device *i2c_bus_master = new Bus_Master_Device("/dev/i2c-0", BUS_TYPE::I2C_BUS);

	BMP085 *bmp085_sensor = new BMP085(0x77);	//BMP085 Sensor with I2C address of 0x77

	Moving_Average_Filter<float> maw_filter(5); //A simple moving average filter with windows size of 5 samples

	int status = 0;
	unsigned int meas_count = 100;

	if (i2c_bus_master->open_bus() < 0)
	{
		std::cout << "Failed to open master bus" << std::endl;
		return -1;
	}

	bmp085_sensor->attach_to_bus(i2c_bus_master);

	status = bmp085_sensor->init_sensor();

	if (status)
	{
		std::cout << "BMP085 sensor initialization failed." << std::endl;
		return status;
	}

	/* Measure the pressure 100 times with 5s sampling time and pass the sample through a moving average filter.
	 * Print the output of the filter on the standard console for each sample.
	 */
	do
	{
		if (bmp085_sensor->measure_temperature_pressure())
		{
			std::cout << "Failed to retrieve sensor data." << std::endl;
			return -1;
		}

		std::cout << "Pressure: P = " << maw_filter.output(bmp085_sensor->temperature_reading()) << "hPa" << std::endl;

		std::this_thread::sleep_for (std::chrono::seconds(1));

	}while(--meas_count);

	delete bmp085_sensor;
	delete i2c_bus_master;
}
