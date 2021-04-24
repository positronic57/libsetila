 /**
  * @file LPS25H.cpp
  *
  * @brief Implementation of the LPS25H class.
  *
  * @author Goce Boshkovski
  * @date 30 Apr 2016
  *
  * @copyright GNU General Public License v3
  *
  */

#include <stdint.h>

#include "LPS25H.h"

#include "setila_errors.h"
#include "slave_device.h"


int LPS25H::init_sensor(uint8_t CTRL_REG1_value, uint8_t CTRL_REG2_value, uint8_t RES_CONF_value)
{
	uint8_t registryValue;

	// Check the device ID by reading WHO_AM_I register
	if (this->read(LPS25H_WHO_AM_I, &registryValue, 1)) {
		return ERROR_READ_FAILED;
	}
	if (registryValue != LPS25H_ID) {
		return ERROR_WRONG_DEVICE_MODEL;
	}

	// Set the value of the LPS25H_RES_CONF
	if (this->write_byte(LPS25H_RES_CONF, RES_CONF_value)) {
	    	return ERROR_WRITE_FAILED;
	}

	// Set the value of the LPS25H_CTRL_REG2.
	if (this->write_byte(LPS25H_CTRL_REG2, CTRL_REG2_value)) {
		return ERROR_WRITE_FAILED;
	}

	// Set the value of the LPS25H_CTRL_REG1.
	if (this->write_byte(LPS25H_CTRL_REG1, CTRL_REG1_value)) {
		return ERROR_WRITE_FAILED;
	}

	return 0;
}


int LPS25H::init_sensor(void)
{
	uint8_t registry_value = 0x0;

	// Check the device ID by reading WHO_AM_I register
	if (this->read(LPS25H_WHO_AM_I, &registry_value, 1)) {
		return ERROR_READ_FAILED;
	}
	if (registry_value != LPS25H_ID) {
		return ERROR_WRONG_DEVICE_MODEL;
	}

	// Set the pressure and temperature internal average to: AVGT = 16 and AVGP = 32.
	if (this->write_byte(LPS25H_RES_CONF, 0x05)) {
		return ERROR_WRITE_FAILED;
	}

	// Enable FIFO.
	if (this->write_byte(LPS25H_CTRL_REG2, 0x40)) {
		return ERROR_WRITE_FAILED;
	}

	// Activate the FIFO mean mode with average of 32 samples.
	if (this->write_byte(LPS25H_FIFO_CTRL, 0xDF)) {
		return ERROR_WRITE_FAILED;
	}

	// Set the ODR to 25Hz, enable block data update and power on the sensor.
	if (this->write_byte(LPS25H_CTRL_REG1, 0xC4)) {
		return ERROR_WRITE_FAILED;
	}

	return 0;
}


int LPS25H::do_one_shot_measurement()
{
	/* Start a pressure and temperature measurement by writing 0x01 in to a CTR_REG2*/
	if (this->write_byte(LPS25H_CTRL_REG2, 0x01)) {
		return ERROR_WRITE_FAILED;
	}

	return get_sensor_readings();
}


int LPS25H::get_sensor_readings(void)
{
	uint8_t pBuffer[3];
	uint8_t tBuffer[2];
	int16_t temperature=0;
	uint8_t STATUS_REG;
	int wd_counter = SENSOR_READING_WATCHDOG_COUNTER;

	/* Check to see whenever a new pressure sample is available. */
	do
	{
		if (this->read(LPS25H_STATUS_REG, &STATUS_REG, 1))
			return ERROR_READ_FAILED;
		wd_counter--;
	} while (!(STATUS_REG & 2) && wd_counter);
	if (!(STATUS_REG & 2) && !wd_counter) {
		return ERROR_SENSOR_READING_TIME_OUT;
	}

	/* Read pressure registers. MSB bit of LPS25H_PRESS_POUT_XL address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (this->read((LPS25H_PRESS_POUT_XL | 0x80), pBuffer, 3)) {
		return ERROR_READ_FAILED;
	}

	/* Check to see whenever a new temperature sample is available. */
	wd_counter = SENSOR_READING_WATCHDOG_COUNTER;
	do
	{
		if (this->read(LPS25H_STATUS_REG, &STATUS_REG, 1)) {
			return ERROR_READ_FAILED;
		}
		wd_counter--;
	} while (!(STATUS_REG & 1) && wd_counter);
	if (!(STATUS_REG & 1) && !wd_counter) {
		return ERROR_SENSOR_READING_TIME_OUT;
	}

	/* Read temperature registers. MSB bit of LPS25H_TEMP_OUT_L address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (this->read((LPS25H_TEMP_OUT_L | 0x80), tBuffer, 2)) {
		return ERROR_READ_FAILED;
	}

	/*
	 * Calculate the pressure value based on the measurement data.
	 * The formula is taken from ST Application Note AN4450.
	 */
	m_pressure_reading = (float)((((uint32_t)pBuffer[2]) << 16) | (((uint32_t)pBuffer[1]) << 8) | (uint32_t)pBuffer[0]) / (float)4096;

	/* Calculate the temperature value based on the measurement data. */
	temperature = tBuffer[1];
	temperature <<= 8;
	temperature |= tBuffer[0];
	/* Convert negative 2's complement values to native negative value */
	if (temperature & 0x8000) {
		temperature = -((~temperature)+1);
	}

	m_temperature_reading = 42.5 + float(temperature)/float(480.0);

	return 0;
}


int LPS25H::enable_FIFO_MEAN(LPS25H_NBR_AVERAGED_SAMPLES NUM_AVERAGED_SAMPLES)
{
	uint8_t FIFO_CTRL = (1 << FIFO_CTRL_F_MODE2) | (1 << FIFO_CTRL_F_MODE1);		//Enable FIFO_MEAN mode.

	uint8_t CTRL_REG2 = 1 << LPS25H_CTRL_REG2_FIFO_EN;				// Enable FIFO with disabled FIFO_MEAN decimation.

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
			return ERROR_UNSUPPORTED_DEVICE_OPTION_CONFIG;
	}

	/* Send the new value of FIFO_CTRL register to the sensor. */
	if (this->write_byte(LPS25H_FIFO_CTRL, FIFO_CTRL))
		return ERROR_WRITE_FAILED;

	/* Send the new value of CTRL_REG2 register to the sensor. */
	if (this->write_byte(LPS25H_CTRL_REG2, CTRL_REG2))
		return ERROR_WRITE_FAILED;

	return 0;
}


int LPS25H::disable_FIFO_MEAN(void)
{
	uint8_t CTRL_REG2;

	/* Read the current value of CTRL_REG2 register from the sensor. */
	if (this->read(LPS25H_CTRL_REG2, &CTRL_REG2, 1))
		return ERROR_READ_FAILED;

	/* Disable the FIFO_EN bit of CTRL_REG2. */
	CTRL_REG2 &= (~(1 << LPS25H_CTRL_REG2_FIFO_EN));

	/* Write the new value of CTRL_REG2 register in the sensor. */
	if (this->read(LPS25H_CTRL_REG2, &CTRL_REG2, 1))
		return ERROR_READ_FAILED;

	return 0;
}


int LPS25H::power_up(void)
{
	/* Set the value of the PD bit from CTRL_REG1 to 1. */
	if (this->write_byte(LPS25H_CTRL_REG1, (m_CTRL_REG1 | (1 << LPS25H_CTRL_REG1_PD))))
		return ERROR_WRITE_FAILED;

	return 0;
}


int LPS25H::power_down(void)
{
	/* Read the value of the CTRL_REG1 */
	if (this->read(LPS25H_CTRL_REG1, &m_CTRL_REG1, 1))
		return ERROR_READ_FAILED;

	/* Write the new value of the CTRL_REG1 with PD bit set to 0. */
	if (this->write_byte(LPS25H_CTRL_REG1, (m_CTRL_REG1 | (~(1 << LPS25H_CTRL_REG1_PD)))))
		return ERROR_WRITE_FAILED;

	return 0;
}


int LPS25H::set_one_shot_mode(void)
{
	return init_sensor(0x84, 0x00, 0x05);
}


int LPS25H::SW_reset(void)
{
	/* Set the bits BOOT and SWRESET of CTRL_REG2 to 1. */
	uint8_t CTRL_REG2 = (1 << LPS25H_CTRL_REG2_BOOT) | (1 << LPS25H_CTRL_REG2_SWRESET);

	/* Write the new value of the CTRL_REG1 on the sensor. */
	if (this->write_byte(LPS25H_CTRL_REG2, CTRL_REG2))
		return ERROR_WRITE_FAILED;

	return 0;
}
