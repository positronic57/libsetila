/**
 * @file bmp085_bmp180.cpp 
 *
 * @brief Example using libsetila for measuring pressure and temperature with Bosh BMP085/BMP180
 * sensor. The sensor readings are stored in FIFO_queue data container. 
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 */


#include <iostream>

#include "setila/setila_i2c.h"
#include "setila/bdc/FIFO_queue.h"
#include "setila/BMP085.h"

extern "C" {
    #include <unistd.h>     //required for sleep()
};

struct BMP085_readings {
    double temperature;
    double pressure;
    int number;
};

int main()
{

	Bus_Master_Device *i2c_bus_master = new Bus_Master_Device("/dev/i2c-0", BUS_TYPE::I2C_BUS);

	BMP085 *bmp085_sensor = new BMP085(0x77);

    FIFO_queue<struct BMP085_readings> measurement_buffer(30);

    struct BMP085_readings reading;

	int status;
    int meas_count = 0;

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


    while(!measurement_buffer.isFull())
    {
        if (bmp085_sensor->measure_temperature_pressure())
	    {
	    	std::cout << "Failed to retrieve sensor data." << std::endl;
		    return -1;
	    }
	    
        reading.pressure = bmp085_sensor->pressure_reading();
        reading.temperature = bmp085_sensor->temperature_reading();
        reading.number = ++meas_count;

        measurement_buffer.addElement(reading);

        std::cout << "Measurement #" << meas_count << std::endl;
        sleep(2);
    }

    std::cout << "BMP085 readings:" << std::endl;

	std::cout << std::showpos;
	while(!measurement_buffer.isEmpty())
    {
        measurement_buffer.getElement(reading);

        std::cout << "Reading #" << reading.number << std::endl;
        std::cout << "Temperature: t = " << reading.temperature << "°C" << std::endl;
	    std::cout << "Pressure: P = " << reading.pressure << "hPa" << std::endl;
        std::cout << std::endl;
    }

	delete bmp085_sensor;
	delete i2c_bus_master;

	return 0;
}

