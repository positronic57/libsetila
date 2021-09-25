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


int LPS25H::verify_device_id()
{
	uint8_t registryValue;

	// Check the device ID by reading WHO_AM_I register
	if (interface()->read(LPS25H_WHO_AM_I, &registryValue, 1)) {
		return ERROR_READ_FAILED;
	}

	if (registryValue != LPS25H_ID) {
		return ERROR_WRONG_DEVICE_MODEL;
	}

	m_device_id_verified = true;

	return 0;
}


int LPS25H::set_resolution(uint8_t average_1, uint8_t average_2)
{
	//average_1 is the pressure average
	//average_2 is the temperature average
	uint8_t temp = average_1  | (average_2 << 2);

	if (!m_device_id_verified) {
		if (verify_device_id()) {
			return ERROR_WRONG_DEVICE_MODEL;
		}
	}

	// Write the pressure and humidity resolution in AV_CONF register
	if (interface()->write_byte(LPS25H_RES_CONF, temp)) {
		return ERROR_WRITE_FAILED;
	}

	return 0;
}


int LPS25H::set_mode_of_operation(mode_of_operation_t mode_of_operation, output_data_rate_t output_data_rate)
{
	return set_mode_of_operation(mode_of_operation, output_data_rate, LPS25H_NBR_AVERAGED_SAMPLES::AVER_SAMPLES_2);
}


int LPS25H::set_mode_of_operation(mode_of_operation_t mode_of_operation, output_data_rate_t output_data_rate, LPS25H_NBR_AVERAGED_SAMPLES fifo_mean_samples)
{
	if (!m_device_id_verified) {
		if (verify_device_id()) {
			return ERROR_WRONG_DEVICE_MODEL;
		}
	}

	switch(mode_of_operation) {
	case mode_of_operation_t::OP_POWER_DOWN:
		return power_down();
		break;
	case mode_of_operation_t::OP_ONE_SHOT:
		return enable_one_shot_mode();
		break;
	case mode_of_operation_t::OP_CONTINUOUS:
		return config_continuous_mode(output_data_rate);
		break;
	case mode_of_operation_t::OP_FIFO_MEAN_MODE:
		config_continuous_mode(output_data_rate);
		return config_fifo_mean_mode(fifo_mean_samples);
		break;
	case mode_of_operation_t::OP_FIFO_MODE:
	default:
		//TODO Implement FIFO MODE configuration
        return ERROR_UNSUPPORTED_DEVICE_OPTION_CONFIG; 
		break;
	}

	return 0;
}


int LPS25H::enable_one_shot_mode()
{
	uint8_t ctrl_reg1 = m_CTRL_REG1;
	uint8_t ctrl_reg2 = m_CTRL_REG2;

	// Power UP and block continuous data register update
	m_CTRL_REG1 |= ((1 << LPS25H_CTRL_REG1_PD) | (1 << LPS25H_CTRL_REG1_BDU));

	// Set output data rate to 0
	m_CTRL_REG1 &= ~((1 << LPS25H_CTRL_REG1_ODR2) | (1 << LPS25H_CTRL_REG1_ODR1) | (1 << LPS25H_CTRL_REG1_ODR0));

	if (interface()->write_byte(LPS25H_CTRL_REG1, m_CTRL_REG1)) {
		m_CTRL_REG1 = ctrl_reg1;
		return ERROR_WRITE_FAILED;
	}

	// Disable FIFO
	m_CTRL_REG2 &= ~(1 << LPS25H_CTRL_REG2_FIFO_EN);
	if (interface()->write_byte(LPS25H_CTRL_REG2, m_CTRL_REG2)) {
		m_CTRL_REG2 = ctrl_reg2;
		return ERROR_WRITE_FAILED;
	}

	return 0;
}


int LPS25H::do_one_shot_measurement()
{
	uint8_t ctrl_reg2 = m_CTRL_REG2;

	m_CTRL_REG2 |= (1 << LPS25H_CTRL_REG2_ONE_SHOT);

	if (interface()->write_byte(LPS25H_CTRL_REG2, ctrl_reg2)) {
		m_CTRL_REG2 = ctrl_reg2;
		return ERROR_WRITE_FAILED;
	}

	return 0;
}

int LPS25H::get_sensor_readings()
{
	if (!m_device_id_verified) {
		if (verify_device_id()) {
			return ERROR_WRONG_DEVICE_MODEL;
		}
	}

	if (mode_of_operation() == mode_of_operation_t::OP_ONE_SHOT) {
		if (do_one_shot_measurement()) {
			return ERROR_READ_FAILED;
		}
	}

	return read_data_registers();
}


int LPS25H::read_data_registers()
{
	uint8_t pBuffer[3];
	uint8_t tBuffer[2];
	int16_t temperature=0;
	uint8_t STATUS_REG;
	int wd_counter = SENSOR_READING_WATCHDOG_COUNTER;

	/* Check to see whenever a new pressure sample is available. */
	do
	{
		if (interface()->read(LPS25H_STATUS_REG, &STATUS_REG, 1))
			return ERROR_READ_FAILED;
		wd_counter--;
	} while (!(STATUS_REG & 2) && wd_counter);
	if (!(STATUS_REG & 2) && !wd_counter) {
		return ERROR_SENSOR_READING_TIME_OUT;
	}

	/* Read pressure registers. MSB bit of LPS25H_PRESS_POUT_XL address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (interface()->read((LPS25H_PRESS_POUT_XL | 0x80), pBuffer, 3)) {
		return ERROR_READ_FAILED;
	}

	/* Check to see whenever a new temperature sample is available. */
	wd_counter = SENSOR_READING_WATCHDOG_COUNTER;
	do
	{
		if (interface()->read(LPS25H_STATUS_REG, &STATUS_REG, 1)) {
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
	if (interface()->read((LPS25H_TEMP_OUT_L | 0x80), tBuffer, 2)) {
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


int LPS25H::config_fifo_mean_mode(LPS25H_NBR_AVERAGED_SAMPLES NUM_AVERAGED_SAMPLES)
{
	uint8_t fifo_ctrl = m_FIFO_CTRL;

	uint8_t ctrl_reg2 = m_CTRL_REG2;

	// Enable FIFO_MEAN mode
	m_FIFO_CTRL |= (1 << FIFO_CTRL_F_MODE2) | (1 << FIFO_CTRL_F_MODE1);

	// Define the number of averaged samples for FIFO mean mode
	switch(NUM_AVERAGED_SAMPLES)
	{
		case LPS25H_NBR_AVERAGED_SAMPLES::AVER_SAMPLES_2:
			m_FIFO_CTRL |= FIFO_MEAN_MODE_2_SAMPLES;
			break;
		case LPS25H_NBR_AVERAGED_SAMPLES::AVER_SAMPLES_4:
			m_FIFO_CTRL |= FIFO_MEAN_MODE_4_SAMPLES;
			break;
		case LPS25H_NBR_AVERAGED_SAMPLES::AVER_SAMPLES_8:
			m_FIFO_CTRL |= FIFO_MEAN_MODE_8_SAMPLES;
			break;
		case LPS25H_NBR_AVERAGED_SAMPLES::AVER_SAMPLES_16:
			m_FIFO_CTRL |= FIFO_MEAN_MODE_16_SAMPLES;
			break;
		case LPS25H_NBR_AVERAGED_SAMPLES::AVER_SAMPLES_32:
			m_FIFO_CTRL |= FIFO_MEAN_MODE_2_SAMPLES;
			break;
		default:
			return ERROR_UNSUPPORTED_DEVICE_OPTION_CONFIG;
	}

	// Send the new value of FIFO_CTRL register to the sensor
	if (interface()->write_byte(LPS25H_FIFO_CTRL, m_FIFO_CTRL)) {
		m_FIFO_CTRL = fifo_ctrl;
		return ERROR_WRITE_FAILED;
	}

	// Enable FIFO with disabled FIFO_MEAN decimation
	m_CTRL_REG2 |= (1 << LPS25H_CTRL_REG2_FIFO_EN);
	m_CTRL_REG2 &= ~(1 << LPS25H_CTRL_REG2_FIFO_MEAN_DEC);

	// Send the new value of CTRL_REG2 register to the sensor
	if (interface()->write_byte(LPS25H_CTRL_REG2, m_CTRL_REG2)) {
		m_CTRL_REG2 = ctrl_reg2;
		return ERROR_WRITE_FAILED;
	}

	return 0;
}


int LPS25H::power_down(void)
{
	uint8_t ctrl_reg1 = m_CTRL_REG1;

	// Set PD bit to 0
	m_CTRL_REG1 &= ~(1 << LPS25H_CTRL_REG1_PD);

	if (interface()->write_byte(LPS25H_CTRL_REG1, m_CTRL_REG1)) {
		m_CTRL_REG1 = ctrl_reg1;
		return ERROR_WRITE_FAILED;
	}

	return 0;
}


int LPS25H::SW_reset(void)
{
	/* Set the bits BOOT and SWRESET of CTRL_REG2 to 1. */
	uint8_t ctrl_reg2 = m_CTRL_REG2;

	m_CTRL_REG2 |= (1 << LPS25H_CTRL_REG2_BOOT) | (1 << LPS25H_CTRL_REG2_SWRESET);

	/* Write the new value of the CTRL_REG1 on the sensor. */
	if (interface()->write_byte(LPS25H_CTRL_REG2, ctrl_reg2)) {
		m_CTRL_REG2 = ctrl_reg2;
		return ERROR_WRITE_FAILED;
	}


	return 0;
}


int LPS25H::config_continuous_mode(output_data_rate_t output_data_rate)
{
	uint8_t ctrl_reg1 = m_CTRL_REG1;
	uint8_t ctrl_reg2 = m_CTRL_REG2;

	switch (output_data_rate) {
	case output_data_rate_t::ODR_1_Hz:
		m_CTRL_REG1 |= (1 << LPS25H_CTRL_REG1_ODR0);
		m_CTRL_REG1 &= ~(1 << LPS25H_CTRL_REG1_ODR1);
		m_CTRL_REG1 &= ~(1 << LPS25H_CTRL_REG1_ODR2);
		break;
	case output_data_rate_t::ODR_7_Hz:
		m_CTRL_REG1 &= ~(1 << LPS25H_CTRL_REG1_ODR0);
		m_CTRL_REG1 |= (1 << LPS25H_CTRL_REG1_ODR1);
		m_CTRL_REG1 &= ~(1 << LPS25H_CTRL_REG1_ODR2);
		break;
	case output_data_rate_t::ODR_12_5_Hz:
		m_CTRL_REG1 |= (1 << LPS25H_CTRL_REG1_ODR0);
		m_CTRL_REG1 |= (1 << LPS25H_CTRL_REG1_ODR1);
		m_CTRL_REG1 &= ~(1 << LPS25H_CTRL_REG1_ODR2);
		break;
	case output_data_rate_t::ODR_25_Hz:
		m_CTRL_REG1 &= ~(1 << LPS25H_CTRL_REG1_ODR0);
		m_CTRL_REG1 &= ~(1 << LPS25H_CTRL_REG1_ODR1);
		m_CTRL_REG1 |= (1 << LPS25H_CTRL_REG1_ODR2);
		break;
	default:
		return -1;
		break;
	}

	// Do not update the output register until the read is done. Set the power up bit to 1
	m_CTRL_REG1 |= ((1 << LPS25H_CTRL_REG1_BDU) | (1 << LPS25H_CTRL_REG1_PD));

	// Write the output data rate to CTRL_REG1
	if (interface()->write_byte(LPS25H_CTRL_REG1, m_CTRL_REG1)) {
		m_CTRL_REG1 = ctrl_reg1;
		return ERROR_WRITE_FAILED;
	}

	// Disable FIFO
	m_CTRL_REG2 &= ~(1 << LPS25H_CTRL_REG2_FIFO_EN);
	if (interface()->write_byte(LPS25H_CTRL_REG2, m_CTRL_REG2)) {
		m_CTRL_REG2 = ctrl_reg2;
		return ERROR_WRITE_FAILED;
	}

	return 0;

}

