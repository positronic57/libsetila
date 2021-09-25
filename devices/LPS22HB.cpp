/**
 * @file LPS22HB.cpp
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

#include "LPS22HB.h"

#include "setila_errors.h"
#include "slave_device.h"


int LPS22HB::verify_device_id()
{
	uint8_t registryValue;

	// Check the device ID by reading WHO_AM_I register
	if (interface()->read(LPS22HB_WHO_AM_I, &registryValue, 1)) {
		return ERROR_READ_FAILED;
	}

	if (registryValue != LPS22HB_ID) {
		return ERROR_WRONG_DEVICE_MODEL;
	}

	m_device_id_verified = true;

	return 0;
}


int LPS22HB::set_mode_of_operation(mode_of_operation_t mode_of_operation, output_data_rate_t output_data_rate)
{
	if (!m_device_id_verified) {
		if (verify_device_id()) {
			return ERROR_WRONG_DEVICE_MODEL;
		}
	}

	switch(mode_of_operation) {
	case mode_of_operation_t::OP_POWER_DOWN:
	case mode_of_operation_t::OP_ONE_SHOT:
		return enable_one_shot_mode();
		break;
	case mode_of_operation_t::OP_CONTINUOUS:
		return config_continuous_mode(output_data_rate);
		break;
	case mode_of_operation_t::OP_FIFO_MODE:
		//TODO Implement FIFO MODE configuration
        return ERROR_UNSUPPORTED_DEVICE_OPTION_CONFIG;
		break;
	case mode_of_operation_t::OP_FIFO_MEAN_MODE:
        return ERROR_UNSUPPORTED_DEVICE_OPTION_CONFIG;
		break;
	}

	return 0;
}


int LPS22HB::set_resolution(uint8_t average_1, uint8_t average_2)
{
	// Not applicable for LPS22HB
	return 0;
}


int LPS22HB::get_sensor_readings()
{
	if (!m_device_id_verified) {
		if (verify_device_id()) {
			return ERROR_WRONG_DEVICE_MODEL;
		}
	}

	switch (mode_of_operation()) {
	case mode_of_operation_t::OP_POWER_DOWN:
	case mode_of_operation_t::OP_ONE_SHOT:
		if (do_one_shot_measurement()) {
			return ERROR_READ_FAILED;
		}
		break;
	case mode_of_operation_t::OP_CONTINUOUS:
	default:
		return -1;
		break;
	}

	return read_data_registers();
}


int LPS22HB::custom_config(uint8_t &CTRL_REG1_value, uint8_t &CTRL_REG2_value, uint8_t &CTRL_REG3_value)
{
	if (!m_device_id_verified) {
		if (verify_device_id()) {
			return ERROR_WRONG_DEVICE_MODEL;
		}
	}

	uint8_t ctrl_reg1 = m_CTRL_REG1;
	uint8_t ctrl_reg2 = m_CTRL_REG2;
	uint8_t ctrl_reg3 = m_CTRL_REG3;

	m_CTRL_REG3 = CTRL_REG3_value;
	/* Set the value of the LPS22HB_RES_CONF. */
	if (interface()->write_byte(LPS22HB_RES_CONF, m_CTRL_REG3)) {
		m_CTRL_REG3 = ctrl_reg3;
		return ERROR_WRITE_FAILED;
	}

	m_CTRL_REG2 = CTRL_REG2_value;
	/* Set the value of the LPS22HB_CTRL_REG2. */
	if (interface()->write_byte(LPS22HB_CTRL_REG2, m_CTRL_REG2)) {
		m_CTRL_REG2 = ctrl_reg2;
		return ERROR_WRITE_FAILED;
	}

	m_CTRL_REG1 = CTRL_REG1_value;
	/* Set the value of the LPS22HB_CTRL_REG1. */
	if (interface()->write_byte(LPS22HB_CTRL_REG1, m_CTRL_REG1)) {
		m_CTRL_REG1 = ctrl_reg1;
		return ERROR_WRITE_FAILED;
	}

	return 0;
}


int LPS22HB::enable_one_shot_mode(void)
{
	uint8_t ctrl_reg1 = m_CTRL_REG1;
	uint8_t ctrl_reg2 = m_CTRL_REG2;

	// Set all output data rate bits to 0
	m_CTRL_REG1 &= LPS22HB_POWER_DOWN_MODE_DEF;

	// Block the data register updates while reading
	m_CTRL_REG1 |= (1 << LPS22HB_CTRL_REG1_BDU);

	// Write the new CTRL_REG1 value
	if (interface()->write_byte(LPS22HB_CTRL_REG1, m_CTRL_REG1)) {
		m_CTRL_REG1 = ctrl_reg1;
		return ERROR_WRITE_FAILED;
	}

	// Disable internal address incremental for multiple registers reading in one read() call because FIFO stays disabled
	m_CTRL_REG2 &= ~(1 << LPS22HB_CTRL_REG2_IF_ADD_INC);

	if (interface()->write_byte(LPS22HB_CTRL_REG2, m_CTRL_REG2)) {
		m_CTRL_REG2 = ctrl_reg2;
		return ERROR_WRITE_FAILED;
	}

	return 0;
}


int LPS22HB::do_one_shot_measurement(void)
{
	uint8_t ctrl_reg2 = m_CTRL_REG2;

	// Start a pressure and temperature measurement by writing 0x01 in to a CTR_REG2
	m_CTRL_REG2 |= (1 << LPS22HB_CTRL_REG2_ONE_SHOT);

	if (interface()->write_byte(LPS22HB_CTRL_REG2, m_CTRL_REG2)) {
		m_CTRL_REG2 = ctrl_reg2;
		return ERROR_WRITE_FAILED;
	}

	return 0;
}


int LPS22HB::read_data_registers()
{
	uint8_t pBuffer[3];
	uint8_t tBuffer[2];
	int16_t temperature_raw = 0x0;
	bool addr_auto_increment = (m_CTRL_REG2 & (1 << LPS22HB_CTRL_REG2_IF_ADD_INC)) ? true : false;

	int wd_counter = SENSOR_READING_WATCHDOG_COUNTER;
	uint8_t STATUS_REG = 0x00;

	// Check to see whenever a new pressure sample is available
	do
	{
		if (interface()->read(LPS22HB_STATUS_REG, &STATUS_REG, 1)) {
			return ERROR_READ_FAILED;
		}
		wd_counter--;
	} while (!(STATUS_REG & (1 << LPS22HB_STATUS_REG_P_DA)) && wd_counter);

	if (!wd_counter && !(STATUS_REG & (1 << LPS22HB_STATUS_REG_P_DA))) {
		return ERROR_SENSOR_READING_TIME_OUT;
	}


	if (addr_auto_increment) {
		// Register address auto increment is active. Read the 3 pressure registers starting from LPS22HB_PRESS_OUT_XL
		if (interface()->read(LPS22HB_PRESS_OUT_XL, pBuffer, 3)) {
			return ERROR_READ_FAILED;
		}
	}
	else
	{
		// Read the 3 pressure registers separately
		if (interface()->read(LPS22HB_PRESS_OUT_XL, pBuffer, 1)) {
			return ERROR_READ_FAILED;
		}
		if (interface()->read(LPS22HB_PRESS_OUT_L, pBuffer + 1, 1)) {
			return ERROR_READ_FAILED;
		}
		if (interface()->read(LPS22HB_PRESS_OUT_H, pBuffer + 2, 1)) {
			return ERROR_READ_FAILED;
		}

	}

	wd_counter = SENSOR_READING_WATCHDOG_COUNTER;
	// Check to see whenever a new temperature sample is available
	do
	{
		if (interface()->read(LPS22HB_STATUS_REG, &STATUS_REG, 1)) {
			return ERROR_READ_FAILED;
		}
		wd_counter--;
	} while (!(STATUS_REG & (1 << LPS22HB_STATUS_REG_T_DA)) && wd_counter);
	if (!wd_counter && !(STATUS_REG & (1 << LPS22HB_STATUS_REG_T_DA))) {
		return ERROR_SENSOR_READING_TIME_OUT;
	}

	if (addr_auto_increment) {
		// Read the 2 temperature registers in one go. The register address increment is active
		if (interface()->read(LPS22HB_TEMP_OUT_L, tBuffer, 2)) {
			return ERROR_READ_FAILED;
		}
	}
	else
	{
		// Read the 2 temperature registers separately. The register address increment is inactive
		if (interface()->read(LPS22HB_TEMP_OUT_L, tBuffer, 1)) {
			return ERROR_READ_FAILED;
		}
		if (interface()->read(LPS22HB_TEMP_OUT_H, tBuffer + 1, 1)) {
			return ERROR_READ_FAILED;
		}
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


int LPS22HB::config_continuous_mode(output_data_rate_t output_data_rate)
{
	uint8_t ctrl_reg1 = m_CTRL_REG1;
	uint8_t ctrl_reg2 = m_CTRL_REG2;

	switch (output_data_rate) {
	case output_data_rate_t::ODR_1_Hz:
		m_CTRL_REG1 |= (1 << LPS22HB_CTRL_REG1_ODR0);
		m_CTRL_REG1 &= ~(1 << LPS22HB_CTRL_REG1_ODR1);
		m_CTRL_REG1 &= ~(1 << LPS22HB_CTRL_REG1_ODR2);
		break;
	case output_data_rate_t::ODR_10_Hz:
		m_CTRL_REG1 &= ~(1 << LPS22HB_CTRL_REG1_ODR0);
		m_CTRL_REG1 |= (1 << LPS22HB_CTRL_REG1_ODR1);
		m_CTRL_REG1 &= ~(1 << LPS22HB_CTRL_REG1_ODR2);
		break;
	case output_data_rate_t::ODR_25_Hz:
		m_CTRL_REG1 |= (1 << LPS22HB_CTRL_REG1_ODR0);
		m_CTRL_REG1 |= (1 << LPS22HB_CTRL_REG1_ODR1);
		m_CTRL_REG1 &= ~(1 << LPS22HB_CTRL_REG1_ODR2);
		break;
	case output_data_rate_t::ODR_50_Hz:
		m_CTRL_REG1 &= ~(1 << LPS22HB_CTRL_REG1_ODR0);
		m_CTRL_REG1 &= ~(1 << LPS22HB_CTRL_REG1_ODR1);
		m_CTRL_REG1 |= (1 << LPS22HB_CTRL_REG1_ODR2);
		break;
	case output_data_rate_t::ODR_75_Hz:
		m_CTRL_REG1 |= (1 << LPS22HB_CTRL_REG1_ODR0);
		m_CTRL_REG1 &= ~(1 << LPS22HB_CTRL_REG1_ODR1);
		m_CTRL_REG1 |= (1 << LPS22HB_CTRL_REG1_ODR2);
		break;
	case output_data_rate_t::ODR_ONE_SHOT:
		m_CTRL_REG1 &= ~((1 << LPS22HB_CTRL_REG1_ODR1) | (1 << LPS22HB_CTRL_REG1_ODR0));
		m_CTRL_REG1 &= ~(1 << LPS22HB_CTRL_REG1_ODR2);
		break;
	default:
		return -1;
		break;
	}

	// Set BDU to 1 (do not update the output register until the read is done)
	m_CTRL_REG1 |= (1 << LPS22HB_CTRL_REG1_BDU);

	// Disable automatic register address increment
	m_CTRL_REG2 &= ~(1 << LPS22HB_CTRL_REG2_IF_ADD_INC);

	// Write the output data rate to CTRL_REG1
	if (interface()->write_byte(LPS22HB_CTRL_REG1, m_CTRL_REG1)) {
		m_CTRL_REG1 = ctrl_reg1;
		return ERROR_WRITE_FAILED;
	}

	// Write the new value of the CTRL_REG2
	if (interface()->write_byte(LPS22HB_CTRL_REG2, m_CTRL_REG2)) {
		m_CTRL_REG2 = ctrl_reg2;
		return ERROR_WRITE_FAILED;
	}

	return 0;
}
