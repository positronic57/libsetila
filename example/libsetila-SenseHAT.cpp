 /**
  * @file libsetila-SenseHAT.cpp
  * @brief An example of using libsetila for temperature, pressure and humidity measurement with Sense HAT.
  *
  * @author Goce Boshkovski
  * @copyright GNU General Public License v2.
  *
  */

#include <iostream>
#include "I2CBus.h"
#include "LPS25H.h"
#include "HTS221.h"
#include "setila_aux.h"
#include "setila_errors.h"

int main(void)
{
	I2CBus i2cbus("/dev/i2c-1");
	LPS25H LPS25HSensor(0x5C); //Address of the sensor on Sense HAT is 0x5C
	HTS221 HTS221Sensor(0x5F); //Address of the sensor on Sense HAT is 0x5F

	int error_code;

	if (i2cbus.openI2CBus())
	{
		std::cout << "Cannot connect to I2C bus." << std::endl;
		return ERROR_OPEN_I2C_BUS;
	}

	/* Attach the sensors to the I2C bus */
	LPS25HSensor.attachSensorToI2CBus(i2cbus.getI2CBus());
	HTS221Sensor.attachSensorToI2CBus(i2cbus.getI2CBus());

	/* Init LPS25Sensor */
	error_code=LPS25HSensor.initSensor(0x84,0x00,0x00,0x05);

	if(error_code)
	{
		std::cout << "Sensor initialization failed." << std::endl;
		return error_code;
	}

	if (LPS25HSensor.startPressureMeasurement())
	{
		std::cout << "Pressure/temperature measurement failed." << std::endl;
		return error_code;
	}

	std::cout << "LPS25H readings:" << std::endl;
	std::cout << "Pressure P=" << LPS25HSensor.getPressureReading() << "[hPa]" << std::endl;
	std::cout << "Temperature T=" << LPS25HSensor.getTemperaturReading() << "[°C]\n" << std::endl;

#if 0
	/* Init HTS221 Sensor for one shot measurement */
	HTS221Sensor.initSensor(0x1B,0x84,0x00,0x00);
	/* Start a one shot humidity measurement */
	HTS221Sensor.doOneShotMeasurement();
#endif

	/* Init HTS221 Sensor with ODR = 1Hz */
	HTS221Sensor.initSensor(0x1B, 0x85, 0x00, 0x00);
	/* Get the humidity and temperature readings from the sensor and calculate the current temperature and humidity values. */
	HTS221Sensor.getSensorReadings();

	/* Print the humidity and temperature value. */
	std::cout << "HTS221 readings:" << std::endl;
 	std::cout << "Relative Humidity R=" << HTS221Sensor.HumidityReading() << "[%rH]" << std::endl;
	std::cout << "Temperature T=" << HTS221Sensor.TemperatureReading() << "[°C]" << std::endl;

	// Close the access to the I2C bus.
	i2cbus.closeI2CBus();

	return 0;

}
