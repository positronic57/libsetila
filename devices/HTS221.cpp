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

int HTS221::calculate_realtive_humidity()
{
	int16_t H0_T0_out = 0;
	int16_t H1_T0_out = 0;
	int16_t H_T_out = 0;
	float H0_rh, H1_rh;

	H0_rh = m_calibration_table[0]/2.0;
	H1_rh = m_calibration_table[1]/2.0;

	H0_T0_out = m_calibration_table[7];
	H0_T0_out <<= 8;
	H0_T0_out |= m_calibration_table[6];

	H1_T0_out = m_calibration_table[11];
	H1_T0_out <<= 8;
	H1_T0_out |= m_calibration_table[10];

	H_T_out = m_humidity_out[1];
	H_T_out <<= 8;
	H_T_out |= m_humidity_out[0];

	/* convert negative 2's complement values to native negative value */
	if (H0_T0_out&0x8000) H0_T0_out = -((~H0_T0_out) + 1);
	if (H1_T0_out&0x8000) H1_T0_out = -((~H1_T0_out) + 1);
	if (H_T_out&0x8000) H_T_out = -((~H_T_out) + 1);

	m_humidity_reading = ((float)((H_T_out - H0_T0_out) * (H1_rh - H0_rh)) / (float)(H1_T0_out - H0_T0_out)) + H0_rh;

	return 0;
}


int HTS221::do_one_shot_measurement()
{
	/* Start a humidity and temperature measurement */
	if (write_byte(HTS221_CTRL_REG2, 1))
		return ERROR_WRITE_FAILED;

	unsigned char STATUS_REG;

	/* Check to see whenever a new humidity sample is available. */
	do
	{
		if (this->read(HTS221_STATUS_REG, &STATUS_REG, 1))
			return ERROR_READ_FAILED;

	} while (!(STATUS_REG & 2));

#if 0
	/* Wait around 5ms for finishing the measurement and start reading the sensor output */
	tdelay(5000000L);
#endif

	/* Read humidity registers. MSB bit of HTS221_HUMIDITY_OUT_L address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (this->read((HTS221_HUMIDITY_OUT_L | 0x80), m_humidity_out, 2))
		return ERROR_READ_FAILED;

	/* Check to see whenever a new temperature sample is available. */
	do
	{
		if (this->read(HTS221_STATUS_REG, &STATUS_REG, 1))
			return ERROR_READ_FAILED;

	} while (!(STATUS_REG & 1));

	/* Read temperature registers. MSB bit of HTS221_TEMP_OUT_L address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (this->read((HTS221_TEMP_OUT_L | 0x80), m_temperature_out, 2))
		return ERROR_READ_FAILED;

	/* Calculate the relative humidity value based on the measurement data. */
	calculate_realtive_humidity();
	/* Calculate the temperature value based on the measurement data. */
	calculate_temperature();

	return 0;
}

int HTS221::read_calibration_table()
{

	if(this->read((HTS221_CALIB_0 | 0x80), m_calibration_table, 16))
		return 1;

	return 0;
}

int HTS221::init_sensor(unsigned char AV_CONF_value,unsigned char CTRL_REG1_value)
{
	unsigned char registryValue;

	/*Check the device ID by reading WHO_AM_I register*/
	if (this->read(HTS221_WHO_AM_I, &registryValue, 1))
		return ERROR_READ_FAILED;

	if (registryValue != 0xBC)
		return ERROR_WRONG_DEVICE_MODEL;

	/* Read HTS221 calibration table. */
	if (read_calibration_table())
		return ERROR_READ_FAILED;

	/* Set the values of AV_CONF registers. */
	if (this->write_byte(HTS221_AV_CONF, AV_CONF_value))
		return ERROR_WRITE_FAILED;

	/* Set the value of the HTS221_CTRL_REG1. */
	if (this->write_byte(HTS221_CTRL_REG1, CTRL_REG1_value))
		return ERROR_WRITE_FAILED;

	return 0;
}

int HTS221::init_sensor(void)
{
	unsigned char registryValue;

	/*Check the device ID by reading WHO_AM_I register*/
	if (this->read(HTS221_WHO_AM_I, &registryValue, 1))
		return ERROR_READ_FAILED;

	if (registryValue != 0xBC)
		return ERROR_WRONG_DEVICE_MODEL;

	/* Set the pressure and temperature internal average to: AVGT = 32 and AVGP = 64. */
	if (this->write_byte(HTS221_AV_CONF, 0x24))
		return ERROR_WRITE_FAILED;

	/* Set the ODR to 12.5Hz, enable block data update and power on the sensor. */
	if (this->write_byte(HTS221_CTRL_REG1, 0x87))
		return ERROR_WRITE_FAILED;

	return 0;
}

int HTS221::set_one_shot_mode(void)
{
	/* Set one shot mode with the following parameters:
	 * - pressure and temperature internal average values: AVGT = 256 and AVGP = 512;
	 * - block data update bit in CTRL_REG1 set to 1;
	 * - power ON the sensor.
	 */
	return init_sensor(0x3F,0x84);
}

int HTS221::get_sensor_readings(void)
{
	unsigned char STATUS_REG;

	/* Check to see whenever a new humidity sample is available. */
	do
	{
		if (this->read(HTS221_STATUS_REG, &STATUS_REG, 1))
			return ERROR_READ_FAILED;

	} while (!(STATUS_REG & 2));

	/* Read humidity registers. MSB bit of HTS221_HUMIDITY_OUT_L address is set to 1 forr
	 * enabling address auto-increment.
	 */
	if (this->read((HTS221_HUMIDITY_OUT_L | 0x80), m_humidity_out, 2))
		return ERROR_READ_FAILED;

	/* Check to see whenever a new temperature sample is available. */
	do
	{
		if (this->read(HTS221_STATUS_REG, &STATUS_REG, 1))
			return ERROR_READ_FAILED;

	} while (!(STATUS_REG & 1));

	/* Read temperature registers. MSB bit of HTS221_TEMP_OUT_L address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (this->read((HTS221_TEMP_OUT_L | 0x80), m_temperature_out, 2))
		return ERROR_READ_FAILED;

	/* Calculate the relative humidity value based on the measurement data. */
	calculate_realtive_humidity();
	/* Calculate the temperature value based on the measurement data. */
	calculate_temperature();

	return 0;
}

int HTS221::power_down(void)
{
	/* Read the value of the CTRL_REG1 */
	if (this->read(HTS221_CTRL_REG1, &CTRL_REG1, 1))
		return ERROR_READ_FAILED;

	/* Write the new value of the CTRL_REG1 with PD bit set to 0. */
	if (this->write_byte(HTS221_CTRL_REG1, (CTRL_REG1 | (~(1 << HTS221_CTRL_REG1_PD)))))
		return ERROR_WRITE_FAILED;

	return 0;
}

int HTS221::power_up(void)
{
	/* Set the value of the PD bit from CTRL_REG1 to 1. */
	if (this->write_byte(HTS221_CTRL_REG1, (CTRL_REG1 | (1 << HTS221_CTRL_REG1_PD))))
		return ERROR_WRITE_FAILED;

	return 0;
}


