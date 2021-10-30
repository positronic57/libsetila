/**
* @file mcp9808.cpp
*
* @brief Implementation of MCP9808 class.
*
* @author Goce Boshkovski
* @date 23 Oct 2021
*
* @copyright GNU General Public License v3
*/


#include "mcp9808.h"
#include "setila_errors.h"


int MCP9808::verify_device_id()
{
   uint16_t registryValue = 0;

   // Check the device ID by reading DEVICE ID register
   if (read(MCP9808_DEV_ID_REG, &registryValue, 2)) {
     return ERROR_READ_FAILED;
   }

    // Device ID is the MSB of the DEVICE ID register
   if ((registryValue & 0x00FF) != MCP9808_DEV_ID) {
	 return ERROR_WRONG_DEVICE_MODEL;
   }

   m_device_id_verified = true;

   return 0;
}


int MCP9808::set_mod_of_operation(const MCP9808::MODE_OF_OPERATION op_mode, const MCP9808::RESOLUTION resolution)
{
    int status = 0;

    if (!m_device_id_verified) {
      if (verify_device_id()) {
        return ERROR_WRONG_DEVICE_MODEL;
      }
    }

    switch(op_mode) {
        case MODE_OF_OPERATION::CONTINUOUS_CONVERSION:
        status = continuous_conversion();
        break;
    case MODE_OF_OPERATION::SHUTDOWN:
		status = shutdown();
        break;
	default:
		break;
	}

	if (status) {
      return status;
    }

    return set_resolution(resolution);
}


int MCP9808::get_sensor_readings()
{
    uint16_t ambient_temperature = 0;
    uint8_t ambient_temperature_msb = 0;
    uint8_t ambient_temperature_lsb = 0;

    if (!m_device_id_verified) {
	  if (verify_device_id()) {
        return ERROR_WRONG_DEVICE_MODEL;
	  }
    }

    if (this->read(MCP9808_TEMPERATURE_REG, &ambient_temperature, sizeof(ambient_temperature))) {
       return ERROR_READ_FAILED;
    }

    ambient_temperature_lsb = (ambient_temperature & 0xFF00) >> 8;

    ambient_temperature_msb = ambient_temperature & 0x00FF;

    // Clear flag bits
    ambient_temperature_msb &= 0x1F;

    if ((ambient_temperature_msb & 0x10) == 0x10) {
        // Ambient temperature < 0°C, clear sign
        ambient_temperature_msb &= 0x0F;
        m_temperature = 256.0 - (ambient_temperature_msb * 16.0 + ambient_temperature_lsb / 16.0);
    }
    else {
        // Ambient temperature ≥ 0°C
        m_temperature = (ambient_temperature_msb * 16.0 + ambient_temperature_lsb / 16.0);
    }

    return 0;
}


int MCP9808::shutdown()
{
  uint16_t temp = m_config_register;

  // Set shutdown bit to value 1
  m_config_register |= (1 << MCP9808_CONGIF_REG_SHDN);

  // Write the new Control register value
  if (this->write(MCP9808_CONFIG_REG, &m_config_register, sizeof(m_config_register))) {
     m_config_register = temp;
     return ERROR_WRITE_FAILED;
   }

   return 0;
}


int MCP9808::continuous_conversion()
{
   uint16_t temp = m_config_register;

   // Set shutdown bit to 0
   m_config_register &= ~(1 << MCP9808_CONGIF_REG_SHDN);

   // Write the new Control register value
   if (this->write(MCP9808_CONFIG_REG, &m_config_register, sizeof(m_config_register))) {
	 m_config_register = temp;
	 return ERROR_WRITE_FAILED;
   }

   return 0;
}


int MCP9808::set_resolution(const MCP9808::RESOLUTION resolution)
{
    uint8_t res = static_cast<uint8_t>(resolution);

    if (this->write(MCP9808_RESOLUTION_REG, &res, sizeof(res))) {
	return ERROR_WRITE_FAILED;
    }

    return 0;
}
