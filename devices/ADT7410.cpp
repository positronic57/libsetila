/**
* @file ADT7410.cpp
*
* @brief Implementation of ADT7410 class.
*
* @author Goce Boshkovski
* @date 1 Nov 2021
*
* @copyright GNU General Public License v3
*/

#include <stdint.h>
#include "ADT7410.h"

#include "setila_errors.h"

int ADT7410::set_mode_of_operation(const ADT7410::MODE_OF_OPERATION mode, const ADT7410::RESOLUTION resolution)
{
	uint8_t temp_control_reg = m_config_register;

    switch(mode) {
    case MODE_OF_OPERATION::CONTINUOUS_CONVERSION:
        m_config_register &= ~(1 << ADT7410_CONFIG_REG_OP_BIT1);
        m_config_register &= ~(1 << ADT7410_CONFIG_REG_OP_BIT2);
        break;
    case MODE_OF_OPERATION::ONE_SHOT:
    	m_config_register |= (1 << ADT7410_CONFIG_REG_OP_BIT1);
    	m_config_register &= ~(1 << ADT7410_CONFIG_REG_OP_BIT2);
    	break;
    case MODE_OF_OPERATION::ONE_SAMPLE_PER_SECOND:
    	m_config_register &= ~(1 << ADT7410_CONFIG_REG_OP_BIT1);
    	m_config_register |= (1 << ADT7410_CONFIG_REG_OP_BIT1);
    	break;
    case MODE_OF_OPERATION::SHUTDOWN:
    	m_config_register |= (1 << ADT7410_CONFIG_REG_OP_BIT1);
    	m_config_register |= (1 << ADT7410_CONFIG_REG_OP_BIT1);
        break;
	default:
		break;
	}

    if (set_resolution(resolution)) {
    	m_config_register = temp_control_reg;
    	return ERROR_WRITE_FAILED;
    }

    return 0;
}


int ADT7410::set_resolution(const ADT7410::RESOLUTION resolution)
{
	uint8_t temp_control_register = m_config_register;

	if (resolution == ADT7410::RESOLUTION::RES_0_0625) {
		m_config_register &= ~(1 << ADT7410_CONFIG_REG_RESOLUTION_BIT);
	}
	else if (resolution == ADT7410::RESOLUTION::RES_0_0078) {
		m_config_register |= (1 << ADT7410_CONFIG_REG_RESOLUTION_BIT);
	}

	if (this->write_byte(ADT7410_CONFIG_REG, m_config_register)) {
		m_config_register = temp_control_register;
		return ERROR_WRITE_FAILED;
	}

	m_resolution = resolution;

	return 0;
}

int ADT7410::get_sensor_readings()
{
	int wd_counter = SENSOR_READING_WATCHDOG_COUNTER * 5;
	uint8_t status_reg = 0x00;
	uint8_t temperature_regs[2] = { 0x00 };
	uint16_t temperature_value = 0;

	// Check to see whenever a new temperature sample is available
	do
	{
		if (this->read(ADT7410_STATUS_REG, &status_reg, 1)) {
			return ERROR_READ_FAILED;
		}

		wd_counter--;

	} while ((status_reg & (1 << ADT7410_STATUS_REG_RDY_BIT)) && wd_counter);

	if ((status_reg & (1 << ADT7410_STATUS_REG_RDY_BIT)) && !wd_counter) {
		return ERROR_SENSOR_READING_TIME_OUT;
	}

	// Read the 2 temperature registers
	if (this->read(ADT7410_STATUS_REG, temperature_regs, 2))
		return ERROR_READ_FAILED;

	// Swap the MSB and LSB from the temperature reading
	temperature_value = (uint16_t) (temperature_regs[0] << 8);
	temperature_value |= temperature_regs[1];

	if (m_resolution == ADT7410::RESOLUTION::RES_0_0625)
	{
		//13-bit temperature value
		if (temperature_value & 0x8000) // Negative temperature
		{
			m_temperature = (float)((int16_t)temperature_value - 65536) / 128;
		}
		else
		{
			// Positive temperature
			m_temperature = (float)temperature_value / 128;
		}
	}
	else if (m_resolution == ADT7410::RESOLUTION::RES_0_0078)
	{
		//16-bit temperature value
		temperature_value >>= 3;
		if (temperature_value & 0x1000)	// Negative temperature
		{
			m_temperature = (float)((signed long)temperature_value - 8192) / 16;
		}
		else
		{
			// Positive temperature
			m_temperature = (float)temperature_value / 16;
		}
	}

	return 0;
}
