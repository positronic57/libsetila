 /**
  * @file LPS25H.cpp
  * @brief Implementation of the LPS25H class.
  *
  * @author Goce Boshkovski
  * @date 30-Apr-16
  * @copyright GNU General Public License v2.
  *
  */

#include <cinttypes>
#include "LPS25H.h"
#include "setila_errors.h"

LPS25H::LPS25H(unsigned char SensorAddress):I2CSensor(SensorAddress),pressureReading(0.0),temperatureReading(0.0){}

LPS25H::~LPS25H() {
}

float LPS25H::getPressureReading()
{
	return pressureReading;
}

float LPS25H::getTemperaturReading()
{
	return temperatureReading;
}

int LPS25H::initSensor(unsigned char CTRL_REG1_value,unsigned char CTRL_REG2_value,unsigned char CTRL_REG3_value,unsigned char RES_CONF_value)
{
	unsigned char registryValue;

	/*Check the device ID by reading WHO_AM_I register*/
	if (I2CSensor_Read(LPS25H_WHO_AM_I,&registryValue,1))
		return ERROR_INIT_LPS25H_SENSOR;
	if (registryValue!=0xBD)
		return ERROR_INIT_LPS25H_SENSOR;

	/* Set the value of the LPS25H_RES_CONF. */
	if (I2CSensor_Write(LPS25H_RES_CONF,RES_CONF_value))
	    	return ERROR_INIT_LPS25H_SENSOR;
	/* Set the value of the LPS25H_CTRL_REG1. */
	if (I2CSensor_Write(LPS25H_CTRL_REG1,CTRL_REG1_value))
		return ERROR_INIT_LPS25H_SENSOR;

	return 0;
}

int LPS25H::startPressureMeasurement()
{
	unsigned char pBuffer[3];
	unsigned char tBuffer[2];
	int16_t temperature=0;
	int32_t pressure=0;

	/* Start a pressure and temperature measurement by writing 0x01 in to a CTR_REG2*/
	if (I2CSensor_Write(LPS25H_CTRL_REG2,0x01))
		return ERROR_LPS25H_MEASUREMENT_FAILED;

	/* Wait around 5ms for finishing the measurement and start reading the sensor output. */
	tdelay(5000000L);

	/* Read pressure registers. MSB bit of LPS25H_PRESS_POUT_XL address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (I2CSensor_Read((LPS25H_PRESS_POUT_XL | 0x80),pBuffer,3))
		return ERROR_LPS25H_MEASUREMENT_FAILED;

	/* Read temperature registers. MSB bit of LPS25H_TEMP_OUT_L address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (I2CSensor_Read((LPS25H_TEMP_OUT_L | 0x80),tBuffer,2))
		return ERROR_LPS25H_MEASUREMENT_FAILED;

	/* Calculate the pressure value based on the measurement data. */
	pressure=pBuffer[2];
	pressure<<=8;
	pressure|=pBuffer[1];
	pressure<<=8;
	pressure|=pBuffer[0];
	pressureReading=pressure/4096.0;

	/* Calculate the temperature value based on the measurement data. */
	temperature=tBuffer[1];
	temperature<<=8;
	temperature|=tBuffer[0];
	/* Convert negative 2's complement values to native negative value */
	if (temperature & 0x8000) temperature = -((~temperature)+1);
	temperatureReading = 42.5 + temperature/480.0;

	return 0;
}


