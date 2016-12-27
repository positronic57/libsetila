 /**
  * @file libsetila-SenseHAT.cpp
  * @brief An example of using libsetila for temperature, pressure and humidity measurement with Raspberry Pi SenseHAT.
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

extern "C" {
#include <unistd.h>
}
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

	/* Init LPS25H sensor with some default parameters defined in initSensor():
	 * - output data rate ODR = 25Hz;
	 * - block data update bit BDU = 1;
	 * - pressure internal average AVGP = 32;
	 * - temperature internal average AVRT = 16;
	 * - FIFO enabled, decimation disabled;
	 * - FIFO mean mode enabled with average on 32 samples.
	 */
	LPS25HSensor.initSensor();

	if (LPS25HSensor.getSensorReadings())
	{
		std::cout << "Pressure/temperature measurement failed." << std::endl;
		return error_code;
	}

	std::cout.precision(10);
	std::cout << "LPS25H readings:" << std::endl;
	std::cout << "Pressure P=" << LPS25HSensor.PressureReading() << "[hPa]" << std::endl;
	std::cout << "Temperature T=" << LPS25HSensor.TemperaturReading() << "[°C]\n" << std::endl;

#if 0
	/* Init HTS221 Sensor for one shot measurement */
	HTS221Sensor.setOneShotMode();
	/* Start a one shot humidity measurement */
	HTS221Sensor.doOneShotMeasurement();
#endif

	/* Init HTS221 Sensor with ODR = 1Hz */
	HTS221Sensor.initSensor(0x1B, 0x85);

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
