 /**
  * @file LPS22HB.cpp
  *
  * @brief Implementation of the LPS22HB class.
  *
  * @author Goce Boshkovski
  * @date 17 Apr 2021
  *
  * @copyright GNU General Public License v3
  *
  */

#include <stdint.h>

#include "LPS22HB.h"

#include "setila_errors.h"
#include "slave_device.h"


int LPS22HB::init_sensor(uint8_t &CTRL_REG1_value, uint8_t &CTRL_REG2_value, uint8_t &CTRL_REG_3_value)
{
	uint8_t registryValue;

	/*Check the device ID by reading WHO_AM_I register*/
	if (this->read(LPS22HB_WHO_AM_I, &registryValue, 1)) {
		return ERROR_READ_FAILED;
	}

	if (registryValue != LPS22HB_ID) {
		return ERROR_WRONG_DEVICE_MODEL;
	}

	/* Set the value of the LPS22HB_RES_CONF. */
	if (this->write_byte(LPS22HB_RES_CONF, CTRL_REG_3_value))
	    	return ERROR_WRITE_FAILED;

	/* Set the value of the LPS22HB_CTRL_REG2. */
	if (this->write_byte(LPS22HB_CTRL_REG2, CTRL_REG2_value))
		return ERROR_WRITE_FAILED;

	/* Set the value of the LPS22HB_CTRL_REG1. */
	if (this->write_byte(LPS22HB_CTRL_REG1, CTRL_REG1_value))
		return ERROR_WRITE_FAILED;

	return 0;
}


int LPS22HB::init_sensor(void)
{
	uint8_t registryValue;

	// Check the device ID by reading WHO_AM_I register
	if (this->read(LPS22HB_WHO_AM_I, &registryValue, 1)) {
		return ERROR_READ_FAILED;
	}
	if (registryValue != LPS22HB_ID) {
		return ERROR_WRONG_DEVICE_MODEL;	//Not LPS22HB device
	}

	// Set device in power-down mode. Measurements are done in one-shot mode
	if (enable_one_shot_mode()) {
		return ERROR_WRITE_FAILED;
	}

	// Get the current value of CTRL_REG2 register
	if (this->read(LPS22HB_CTRL_REG2, &m_CTRL_REG2, 1)) {
		return ERROR_READ_FAILED;
	}
	// Enable internal address incremental for multiple registers reading in one read() call
	m_CTRL_REG2 |= (1 << LPS22HB_CTRL_REG2_IF_ADD_INC);
	if (this->write_byte(LPS22HB_CTRL_REG2, m_CTRL_REG2)) {
		return ERROR_WRITE_FAILED;
	}

	return 0;
}


int LPS22HB::do_one_shot_measurement(float &pressure, float &temperature)
{
	int reading_status = 0;

	// Start a pressure and temperature measurement by writing 0x01 in to a CTR_REG2
	m_CTRL_REG2 |= (1 << LPS22HB_CTRL_REG2_ONE_SHOT);
	if (this->write_byte(LPS22HB_CTRL_REG2, m_CTRL_REG2)) {
		return ERROR_WRITE_FAILED;
	}

	reading_status = get_sensor_readings();
	if (reading_status == 0)
	{
		pressure = m_pressure_reading;
		temperature = m_temperature_reading;
	}

	return reading_status;
}

int LPS22HB::get_sensor_readings()
{
	uint8_t pBuffer[3];
	uint8_t tBuffer[2];
	int16_t temperature_raw = 0x0;

	int wd_counter = SENSOR_READING_WATCHDOG_COUNTER;
	uint8_t STATUS_REG = 0x00;

	// Check to see whenever a new pressure sample is available
	do
	{
		if (this->read(LPS22HB_STATUS_REG, &STATUS_REG, 1)) {
			return ERROR_READ_FAILED;
		}
		wd_counter--;
	} while (!(STATUS_REG & (1 << LPS22HB_STATUS_REG_P_DA)) && wd_counter);

	if (!wd_counter && !(STATUS_REG & (1 << LPS22HB_STATUS_REG_P_DA))) {
		return ERROR_SENSOR_READING_TIME_OUT;
	}

	// Read the 3 pressure registers starting from LPS22HB_PRESS_OUT_XL. The register address increment is active
	if (this->read(LPS22HB_PRESS_OUT_XL, pBuffer, 3)) {
		return ERROR_READ_FAILED;
	}

	wd_counter = SENSOR_READING_WATCHDOG_COUNTER;
	// Check to see whenever a new temperature sample is available
	do
	{
		if (this->read(LPS22HB_STATUS_REG, &STATUS_REG, 1)) {
			return ERROR_READ_FAILED;
		}
		wd_counter--;
	} while (!(STATUS_REG & (1 << LPS22HB_STATUS_REG_T_DA)) && wd_counter);
	if (!wd_counter && !(STATUS_REG & (1 << LPS22HB_STATUS_REG_T_DA))) {
		return ERROR_SENSOR_READING_TIME_OUT;
	}

	// Read the 2 temperature registers. The register address increment is active
	if (this->read(LPS22HB_TEMP_OUT_L, tBuffer, 2)) {
		return ERROR_READ_FAILED;
	}

	/*
	 * Calculate the pressure value based on the measurement data.
	 * The formula is taken from ST Application Note AN4450.
	 */
	m_pressure_reading =
			(float)((((uint32_t)pBuffer[2]) << 16) | (((uint32_t)pBuffer[1]) << 8) | (uint32_t)pBuffer[0]) / (float)4096;

	// Calculate the temperature value based on the measurement data
	temperature_raw = tBuffer[1];
	temperature_raw <<= 8;
	temperature_raw |= tBuffer[0];

	// Convert negative 2's complement values to native negative value
	if (temperature_raw & 0x8000) {
		temperature_raw = -((~temperature_raw) + 1);
	}

	m_temperature_reading = float(temperature_raw) / float(100.0);

	return 0;
}

int LPS22HB::enable_one_shot_mode(void)
{
	// Read the value of the CTRL_REG1
	if (this->read(LPS22HB_CTRL_REG1, &m_CTRL_REG1, 1)) {
		return ERROR_READ_FAILED;
	}

	// Set all output data rate bits to 0
	m_CTRL_REG1 &= LPS22HB_POWER_DOWN_MODE_DEF;

	// Write the new CTRL_REG1 value
	if (this->write_byte(LPS22HB_CTRL_REG1, m_CTRL_REG1)) {
		return ERROR_WRITE_FAILED;
	}

	return 0;
}


int LPS22HB::SW_reset(void)
{
	// Read the value of the CTRL_REG2
	if (this->read(LPS22HB_CTRL_REG1, &m_CTRL_REG2, 1)) {
		return ERROR_READ_FAILED;
	}

	// Set the bits BOOT and SWRESET of CTRL_REG2 to 1
	m_CTRL_REG2 |= ((1 << LPS22HB_CTRL_REG2_BOOT) | (1 << LPS22HB_CTRL_REG2_SWRESET));

	// Write the new value of the CTRL_REG1 on the sensor
	if (this->write_byte(LPS22HB_CTRL_REG2, m_CTRL_REG2)) {
		return ERROR_WRITE_FAILED;
	}

	return 0;
}
