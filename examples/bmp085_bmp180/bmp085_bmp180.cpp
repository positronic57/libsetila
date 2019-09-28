#include <iostream>

#include "setila/setila_i2c.h"
#include "setila/BMP085.h"

int main()
{

	Bus_Master_Device *i2c_bus_master = new Bus_Master_Device("/dev/i2c-0", BUS_TYPE::I2C_BUS);

	BMP085 *bmp085_sensor = new BMP085(0x77);

	int status;

	if (i2c_bus_master->open_bus() < 0)
	{
		std::cout << "Failed to open master bus" << std::endl;
		return -1;
	}

	status = bmp085_sensor->attach_to_bus(i2c_bus_master);

	status = bmp085_sensor->init_sensor();

	if (status)
	{
		std::cout << "BMP085 sensor initialization failed." << std::endl;
		return status;
	}

	std::cout << "BMP085 readings:" << std::endl;
	if (bmp085_sensor->measure_temperature_pressure())
	{
		std::cout << "Failed to retrieve sensor data." << std::endl;
		return -1;
	}
	
	std::cout << std::showpos;
	std::cout << "Temperature: t = " << bmp085_sensor->temperature_reading() << "°C" << std::endl;
	std::cout << "Pressure: P = " << bmp085_sensor->pressure_reading() << "hPa" << std::endl;
	
	delete bmp085_sensor;
	delete i2c_bus_master;

	return 0;
}

