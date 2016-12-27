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


LPS25H::LPS25H(unsigned char SensorAddress):I2CSensor(SensorAddress),pressureReading(0.0),temperatureReading(0.0), CTRL_REG1(0x00){}


LPS25H::~LPS25H() {}


float LPS25H::PressureReading()
{
	return pressureReading;
}


float LPS25H::TemperaturReading()
{
	return temperatureReading;
}


int LPS25H::initSensor(unsigned char CTRL_REG1_value, unsigned char CTRL_REG2_value, unsigned char RES_CONF_value)
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

	/* Set the value of the LPS25H_CTRL_REG2. */
	if (I2CSensor_Write(LPS25H_CTRL_REG2,CTRL_REG2_value))
		return ERROR_INIT_LPS25H_SENSOR;

	/* Set the value of the LPS25H_CTRL_REG1. */
	if (I2CSensor_Write(LPS25H_CTRL_REG1,CTRL_REG1_value))
		return ERROR_INIT_LPS25H_SENSOR;

	return 0;
}


int LPS25H::initSensor(void)
{
	/* Set the pressure and temperature internal average to: AVGT = 16 and AVGP = 32. */
	if (I2CSensor_Write(LPS25H_RES_CONF, 0x05))
		return ERROR_INIT_LPS25H_SENSOR;

	/* Enable FIFO. */
	if (I2CSensor_Write(LPS25H_CTRL_REG2, 0x40))
		return ERROR_INIT_LPS25H_SENSOR;

	/* Activate the FIFO mean mode with average of 32 samples. */
	if (I2CSensor_Write(LPS25H_FIFO_CTRL, 0xDF))
		return ERROR_INIT_LPS25H_SENSOR;

	/* Set the ODR to 25Hz, enable block data update and power on the sensor. */
	if (I2CSensor_Write(LPS25H_CTRL_REG1, 0xC4))
		return ERROR_INIT_LPS25H_SENSOR;

	return 0;
}


int LPS25H::doOneShotMeasurement()
{
	unsigned char pBuffer[3];
	unsigned char tBuffer[2];
	int16_t temperature=0;

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

	/*
	 * Calculate the pressure value based on the measurement data.
	 * The formula is taken from ST Application Note AN4450.
	 */
	pressureReading = (float)((((uint32_t)pBuffer[2]) << 16) | (((uint32_t)pBuffer[1]) << 8) | (uint32_t)pBuffer[0]) / (float)4096;

	/* Calculate the temperature value based on the measurement data. */
	temperature=tBuffer[1];
	temperature<<=8;
	temperature|=tBuffer[0];
	/* Convert negative 2's complement values to native negative value */
	if (temperature & 0x8000) temperature = -((~temperature)+1);
	temperatureReading = 42.5 + float(temperature)/float(480.0);

	return 0;
}


int LPS25H::getSensorReadings(void)
{
	unsigned char pBuffer[3];
	unsigned char tBuffer[2];
	int16_t temperature=0;
	unsigned char STATUS_REG;

	/* Check to see whenever a new pressure sample is available. */
	do
	{
		if (I2CSensor_Read(LPS25H_STATUS_REG, &STATUS_REG, 1))
			return ERROR_LPS25H_MEASUREMENT_FAILED;

	} while (!(STATUS_REG & 2));

	/* Read pressure registers. MSB bit of LPS25H_PRESS_POUT_XL address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (I2CSensor_Read((LPS25H_PRESS_POUT_XL | 0x80),pBuffer,3))
		return ERROR_LPS25H_MEASUREMENT_FAILED;

	/* Check to see whenever a new temperature sample is available. */
	do
	{
		if (I2CSensor_Read(LPS25H_STATUS_REG, &STATUS_REG, 1))
			return ERROR_LPS25H_MEASUREMENT_FAILED;

	} while (!(STATUS_REG & 1));

	/* Read temperature registers. MSB bit of LPS25H_TEMP_OUT_L address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (I2CSensor_Read((LPS25H_TEMP_OUT_L | 0x80),tBuffer,2))
		return ERROR_LPS25H_MEASUREMENT_FAILED;

	/*
	 * Calculate the pressure value based on the measurement data.
	 * The formula is taken from ST Application Note AN4450.
	 */
	pressureReading = (float)((((uint32_t)pBuffer[2]) << 16) | (((uint32_t)pBuffer[1]) << 8) | (uint32_t)pBuffer[0]) / (float)4096;

	/* Calculate the temperature value based on the measurement data. */
	temperature=tBuffer[1];
	temperature<<=8;
	temperature|=tBuffer[0];
	/* Convert negative 2's complement values to native negative value */
	if (temperature & 0x8000) temperature = -((~temperature)+1);
	temperatureReading = 42.5 + float(temperature)/float(480.0);

	return 0;
}


int LPS25H::enableFIFO_MEAN(LPS25H_NBR_AVERAGED_SAMPLES NUM_AVERAGED_SAMPLES)
{
	unsigned char FIFO_CTRL = (1 << FIFO_CTRL_F_MODE2) || (1 << FIFO_CTRL_F_MODE1);		//Enable FIFO_MEAN mode.

	unsigned char CTRL_REG2 = 1 << LPS25H_CTRL_REG2_FIFO_EN;				// Enable FIFO with disabled FIFO_MEAN decimation.

	/* Define the number of averaged samples for FIFO mean mode */
	switch(NUM_AVERAGED_SAMPLES)
	{
		case LPS25H_NBR_AVERAGED_SAMPLES::AVER_SAMPLES_2:
			FIFO_CTRL |= FIFO_MEAN_MODE_2_SAMPLES;
			break;
		case LPS25H_NBR_AVERAGED_SAMPLES::AVER_SAMPLES_4:
			FIFO_CTRL |= FIFO_MEAN_MODE_4_SAMPLES;
			break;
		case LPS25H_NBR_AVERAGED_SAMPLES::AVER_SAMPLES_8:
			FIFO_CTRL |= FIFO_MEAN_MODE_8_SAMPLES;
			break;
		case LPS25H_NBR_AVERAGED_SAMPLES::AVER_SAMPLES_16:
			FIFO_CTRL |= FIFO_MEAN_MODE_16_SAMPLES;
			break;
		case LPS25H_NBR_AVERAGED_SAMPLES::AVER_SAMPLES_32:
			FIFO_CTRL |= FIFO_MEAN_MODE_2_SAMPLES;
			break;
		default:
			return ERROR_LPS25H_NBR_AVERAGED_SAMPLES;
	}

	/* Send the new value of FIFO_CTRL register to the sensor. */
	if (I2CSensor_Write(LPS25H_FIFO_CTRL, FIFO_CTRL))
		return ERROR_LPS25H_ENABLE_FIFO_MEAN;

	/* Send the new value of CTRL_REG2 register to the sensor. */
	if (I2CSensor_Write(LPS25H_CTRL_REG2, CTRL_REG2))
		return ERROR_LPS25H_ENABLE_FIFO_MEAN;

	return 0;
}


int LPS25H::disableFIFO_MEAN(void)
{
	unsigned char CTRL_REG2;

	/* Read the current value of CTRL_REG2 register from the sensor. */
	if (I2CSensor_Read(LPS25H_CTRL_REG2, &CTRL_REG2, 1))
		return ERROR_LPS25H_DISABLE_FIFO_MEAN;

	/* Disable the FIFO_EN bit of CTRL_REG2. */
	CTRL_REG2 &= (~(1 << LPS25H_CTRL_REG2_FIFO_EN));

	/* Write the new value of CTRL_REG2 register in the sensor. */
	if (I2CSensor_Read(LPS25H_CTRL_REG2, &CTRL_REG2, 1))
		return ERROR_LPS25H_DISABLE_FIFO_MEAN;

	return 0;
}


int LPS25H::powerUp(void)
{
	/* Set the value of the PD bit from CTRL_REG1 to 1. */
	if (I2CSensor_Write(LPS25H_CTRL_REG1, (CTRL_REG1 | (1 << LPS25H_CTRL_REG1_PD))))
		return ERROR_I2C_WRITE_FAILED;

	return 0;
}


int LPS25H::powerDown(void)
{
	/* Read the value of the CTRL_REG1 */
	if (I2CSensor_Read(LPS25H_CTRL_REG1, &CTRL_REG1, 1))
		return ERROR_I2C_READ_FAILED;

	/* Write the new value of the CTRL_REG1 with PD bit set to 0. */
	if (I2CSensor_Write(LPS25H_CTRL_REG1, (CTRL_REG1 | (~(1 << LPS25H_CTRL_REG1_PD)))))
		return ERROR_I2C_WRITE_FAILED;

	return 0;
}


int LPS25H::setOneShotMode(void)
{
	return initSensor(0x84, 0x00, 0x05);
}


int LPS25H::SWReset(void)
{
	/* Set the bits BOOT and SWRESET of CTRL_REG2 to 1. */
	unsigned char CTRL_REG2 = (1 << LPS25H_CTRL_REG2_BOOT) | (1 << LPS25H_CTRL_REG2_SWRESET);

	/* Write the new value of the CTRL_REG1 on the sensor. */
	if (I2CSensor_Write(LPS25H_CTRL_REG2, CTRL_REG2))
		return ERROR_LPS25H_SW_RESET;

	return 0;
}
