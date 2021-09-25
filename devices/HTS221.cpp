/**
 * @file HTS221.cpp
 *
 * @brief Implementation of the HTS221 class.
 *
 * @author Goce Boshkovski
 * @date 01 May 2016
 *
 * @copyright GNU General Public License v3
 *
 */

#include <stdint.h>
#include "HTS221.h"

#include "setila_errors.h"


int HTS221::verify_device_id()
{
	uint8_t registryValue;

	// Get the valued of the WHO_AM_I register
	if (interface()->read(HTS221_WHO_AM_I, &registryValue, 1)) {
		return ERROR_READ_FAILED;
	}

	if (registryValue != LTS221_ID) {
		return ERROR_WRONG_DEVICE_MODEL;
	}

	m_device_id_verified = true;

	return 0;
}

int HTS221::set_mode_of_operation(mode_of_operation_t mode_of_operation, output_data_rate_t output_data_rate)
{

	if (!m_device_id_verified) {
		if (verify_device_id()) {
			return ERROR_WRONG_DEVICE_MODEL;
		}
	}

	if (!m_calibration_table_read) {
		if (read_calibration_table()) {
			return ERROR_READ_FAILED;
		}
	}

	switch(mode_of_operation) {
	case mode_of_operation_t::OP_POWER_DOWN:
		return power_down();
	case mode_of_operation_t::OP_ONE_SHOT:
		return config_continuous_mode(output_data_rate_t::ODR_ONE_SHOT);
		break;
	case mode_of_operation_t::OP_CONTINUOUS:
		if (output_data_rate != output_data_rate_t::ODR_ONE_SHOT) {
			return config_continuous_mode(output_data_rate);
		}
		else
		{
			return ERROR_UNSUPPORTED_DEVICE_OPTION_CONFIG;
		}
		break;
    case mode_of_operation_t::OP_FIFO_MEAN_MODE:
        return ERROR_UNSUPPORTED_DEVICE_OPTION_CONFIG;
	case mode_of_operation_t::OP_FIFO_MODE:
	default:
		break;
	}

	return 0;
}

int HTS221::config_continuous_mode(output_data_rate_t output_data_rate)
{
	uint8_t temp = m_CTRL_REG1;

	m_CTRL_REG1 |= (1 << HTS221_CTRL_REG1_PD);

	switch (output_data_rate) {
	case output_data_rate_t::ODR_1_Hz:
		m_CTRL_REG1 |= (1 << HTS221_CTRL_REG1_ODR0);
		m_CTRL_REG1 &= ~(1 << HTS221_CTRL_REG1_ODR1);
		break;
	case output_data_rate_t::ODR_7_Hz:
		m_CTRL_REG1 &= ~(1 << HTS221_CTRL_REG1_ODR0);
		m_CTRL_REG1 |= (1 << HTS221_CTRL_REG1_ODR1);
		break;
	case output_data_rate_t::ODR_12_5_Hz:
		m_CTRL_REG1 |= (1 << HTS221_CTRL_REG1_ODR0);
		m_CTRL_REG1 |= (1 << HTS221_CTRL_REG1_ODR1);
		break;
	case output_data_rate_t::ODR_ONE_SHOT:
		m_CTRL_REG1 &= ~((1 << HTS221_CTRL_REG1_ODR1) | (1 << HTS221_CTRL_REG1_ODR0));
		break;
	default:
		return -1;
		break;
	}

	// Set BDU to 1 (do not update the output register until the read is done)
	m_CTRL_REG1 |= (1 << HTS221_CTRL_REG1_BDU);

	// Write the output data rate to CTR_REG1
	if (interface()->write_byte(HTS221_CTRL_REG1, m_CTRL_REG1)) {
		m_CTRL_REG1 = temp;
		return ERROR_WRITE_FAILED;
	}

	return 0;
}

int HTS221::set_resolution(uint8_t average_1, uint8_t average_2)
{
	//average_1 is the humidity average
	//average_2 is the temperature average
	uint8_t temp = average_1  | (average_2 << 3);

	if (!m_device_id_verified) {
		if (verify_device_id()) {
			return ERROR_WRONG_DEVICE_MODEL;
		}
	}

	// Write the temperature and humidity resolution in AV_CONF register
	if (interface()->write_byte(HTS221_AV_CONF, temp)) {
		return ERROR_WRITE_FAILED;
	}

	if (!m_calibration_table_read) {
		if (read_calibration_table()) {
			return ERROR_READ_FAILED;
		}
	}

	return 0;
}

int HTS221::calculate_temperature()
{
	int16_t T0_out, T1_out, T_out;
	uint16_t tmp;
	float T0_degC, T1_degC;

	T0_out = m_calibration_table[13];
	T0_out <<= 8;
	T0_out |= m_calibration_table[12];

	T1_out = m_calibration_table[15];
	T1_out <<= 8;
	T1_out |= m_calibration_table[14];

	T_out = m_temperature_out[1];
	T_out <<= 8;
	T_out|= m_temperature_out[0];

	/* Convert negative 2's complement values to native negative value */
	if (T0_out&0x8000) T0_out = -((~T0_out) + 1);
	if (T1_out&0x8000) T1_out = -((~T1_out) + 1);
	if (T_out&0x8000) T_out = -((~T_out) + 1);

	/* Calculate the T0_degC coefficient */
	tmp = (m_calibration_table[5]) & 0x03;
	tmp <<= 8;
	tmp |= m_calibration_table[2];
	T0_degC = tmp / 8.0;

	/* Calculate the T1_degC coefficient */
	tmp = m_calibration_table[5] & 0x0C;
	tmp <<= 6;
	tmp |= m_calibration_table[3];
	T1_degC = tmp / 8.0;

	m_temperature_reading = ((float)((T_out - T0_out) * (T1_degC - T0_degC)) / (float)(T1_out - T0_out)) + T0_degC;

	return 0;
}

void HTS221::calculate_realtive_humidity()
{

	int16_t H0_rh, H1_rh = 0;
	int16_t H0_T0_out = 0;
	int16_t H1_T0_out = 0;
	int16_t H_T_out = 0;

	H0_rh = m_calibration_table[H0_rH_x2] >> 1;
	H1_rh = m_calibration_table[H1_rH_x2] >> 1;

	H0_T0_out = (((int16_t)m_calibration_table[H0_T0_OUT_H]) << 8) | ((int16_t)m_calibration_table[H0_T0_OUT_L]);

	H1_T0_out = (((int16_t)m_calibration_table[H1_T0_OUT_H]) << 8) | (int16_t)m_calibration_table[H1_T0_OUT_L];

	H_T_out = (((int16_t)m_humidity_out[1]) << 8) | (int16_t)m_humidity_out[0];

	// Convert negative 2's complement values to native negative value
	if (H0_T0_out & 0x8000) { 
		H0_T0_out = -((~H0_T0_out) + 1);
	}
	if (H1_T0_out & 0x8000) {
		H1_T0_out = -((~H1_T0_out) + 1);
	}
	if (H_T_out & 0x8000) {
		H_T_out = -((~H_T_out) + 1);
	}

	m_humidity_reading = (H_T_out - H0_T0_out) * (H1_rh - H0_rh) / (H1_T0_out - H0_T0_out) + H0_rh;
	if (m_humidity_reading > 100.0) {
		m_humidity_reading = 100.0;
	}
}

int HTS221::do_one_shot_measurement()
{
	if (interface()->write_byte(HTS221_CTRL_REG2, (1 << HTS221_CTRL_REG2_ONE_SHOT))) {
		return ERROR_WRITE_FAILED;
	}

	return 0;
}

int HTS221::read_calibration_table()
{

	if(interface()->read((HTS221_CALIB_0 | 0x80), m_calibration_table, 16)) {
		return ERROR_READ_FAILED;
	}

	return 0;
}

int HTS221::get_sensor_readings(void)
{
	unsigned char STATUS_REG;
	int wd_counter = SENSOR_READING_WATCHDOG_COUNTER;

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
		break;
	}

	// Check if a new humidity sample is available for reading
	do
	{
		if (interface()->read(HTS221_STATUS_REG, &STATUS_REG, 1))
			return ERROR_READ_FAILED;
		wd_counter--;
	} while (!(STATUS_REG & 2) && wd_counter);
	if ((!(STATUS_REG & 2) && !wd_counter)) {
		return ERROR_SENSOR_READING_TIME_OUT;
	}

	// Read humidity registers. MSB bit of HTS221_HUMIDITY_OUT_L address is set to 1 for enabling address auto-increment
	if (interface()->read((HTS221_HUMIDITY_OUT_L | 0x80), m_humidity_out, 2)) {
		return ERROR_READ_FAILED;
	}

	// Check to see whenever a new temperature sample is available
	wd_counter = SENSOR_READING_WATCHDOG_COUNTER;
	do
	{
		if (interface()->read(HTS221_STATUS_REG, &STATUS_REG, 1))
			return ERROR_READ_FAILED;
		wd_counter--;
	} while (!(STATUS_REG & 1) && wd_counter);
	if (!(STATUS_REG & 1) && !wd_counter) {
		return ERROR_SENSOR_READING_TIME_OUT;
	}

	// Read temperature registers. MSB bit of HTS221_TEMP_OUT_L address is set to 1 for enabling address auto-increment
	if (interface()->read((HTS221_TEMP_OUT_L | 0x80), m_temperature_out, 2)) {
		return ERROR_READ_FAILED;
	}

	// Calculate the relative humidity value based on the measurement data
	calculate_realtive_humidity();

	// Calculate the temperature value based on the measurement data
	calculate_temperature();

	return 0;
}

int HTS221::power_down(void)
{
	uint8_t temp = m_CTRL_REG1;

	// Set an output rate of 0
	m_CTRL_REG1 &= ~(1 << HTS221_CTRL_REG1_PD);

	// Write the new CTRL_REG1 value
	if (interface()->write_byte(HTS221_CTRL_REG1, m_CTRL_REG1)) {
		m_CTRL_REG1 = temp;
		return ERROR_WRITE_FAILED;
	}

	return 0;
}
