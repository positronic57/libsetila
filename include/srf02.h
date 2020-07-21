/**
 * @file srf02.h
 *
 * @brief Contains the definition of SRF02 class for communication with SRF02 sonar sensor.
 *
 * @author Goce Boshkovski
 * @date 11 Jul 2019
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef SRF02_H_
#define SRF02_H_

#include <stdint.h>

#include "i2c_slave_device.h"
#include "setila_errors.h"

#define SRF02_RANGING_IN_PROGRESS			0xFF

/**
 * \enum SRF02_Register
 * @brief Internal SRF02 registers with their addresses.
 */
enum class SRF02_Register: uint8_t {
    COMMAND = 0x00,            		/**< Command register. */
    UNUSED,                    		/**< Unused. */
    RANGE_HIGH_BYTE,           		/**< High byte of the range register */
    RANGE_LOW_BYTE,            		/**< Low byte of the range register */
    AUTOTUNE_MINIMUM_HIGH_BYTE,		/**< High byte of the autotune minimum register */
    AUTOTUNE_MINIMUM_LOW_BYTE  		/**< Low byte of the autotune minimum register */
};

/**
 * \enum SRF02_Command
 * @brief Enum class with SRF02 commands (command register values).
 */
enum class SRF02_Command: uint8_t {
    REAL_RANGING_MODE_inch = 0x50,	//!< REAL_RANGING_MODE_inch
    REAL_RANGING_MODE_cm = 0x51,  	//!< REAL_RANGING_MODE_cm
    REAL_RANGING_MODE_us = 0x52, 	//!< REAL_RANGING_MODE_us
    FAKE_RANGING_MODE_inch = 0x56,	//!< FAKE_RANGING_MODE_inch
    FAKE_RANGING_MODE_cm = 0x57,  	//!< FAKE_RANGING_MODE_cm
    FAKE_RANGING_MODE_us = 0x58,  	//!< FAKE_RANGING_MODE_us
    PING_NO_RANGING = 0x5C,       	//!< PING_NO_RANGING
    FORCE_AUTOTUNE = 0x60,        	//!< FORCE_AUTOTUNE
    CHANGE_I2C_ADDR_STEP1 = 0xA0, 	//!< CHANGE_I2C_ADDR_STEP1
    CHANGE_I2C_ADDR_STEP2 = 0xA5, 	//!< CHANGE_I2C_ADDR_STEP2
    CHANGE_I2C_ADDR_STEP3 = 0xAA  	//!< CHANGE_I2C_ADDR_STEP3
};

/**
 * \enum srf02_status_t
 * @brief Return codes for the SRF02 functions.
 */
typedef enum {
    SRF02_READ_WRITE_ERROR = -5,   		//!< SRF02_READ_WRITE_ERROR
    SRF02_UNSUPPORTED_COMMAND = -4,		//!< SRF02_UNSUPPORTED_COMMAND
	SRF02_OBJECT_FOUND = -3,          	//!< SRF02_OBJECT_FOUND
	SRF02_NO_OBJECT_IN_RANGE = -2,    	//!< SRF02_NO_OBJECT_IN_RANGE
	SRF02_READING_RANGE_FAILED = -1,  	//!< SRF02_READING_RANGE_FAILED
	SRF02_OPERATION_SUCCESSFUL = 0    	//!< SRF02_OPERATION_SUCCESSFUL
} srf02_status_t;

/**
 * \class SRF02
 * @ingroup I2C_SLAVE_DEVICES
 * @brief The class represents SRF02 ultra sonic range finder
 *
 * @example range_finder.cpp
 */
class SRF02 : public I2C_Slave_Device
{
private:
	uint8_t m_software_version = 0x00;		/**< Holds the firmware version of the sensor read from the internal sensor registers. */
	uint16_t m_distance = 0x00;				/**< Holds the distance value measured by the sensor. */

public:
	SRF02(): I2C_Slave_Device(0x70) {};

	/**
	 * @brief Class constructor.
	 *
	 * @param[in] i2c_slave_address	Address of the slave device on I2C bus.
	 */
	explicit SRF02(uint8_t i2c_slave_address): I2C_Slave_Device(i2c_slave_address) {};

	/**
	 * @brief Class destructor.
	 */
	~SRF02() {};

	/**
	 * @brief returns the value of the class member m_distance
	 * @return distance value
	 */
	const uint16_t distance(void) const { return m_distance; };

	/**
	 * @brief Starts the ranging process.
	 *
	 * Current version sends a command for ranging result in [cm].
	 *
	 * @return request status of type srf02_status_t. SRF02_OBJECT_FOUND in case of a successful data exchange with SRF02, status code otherwise
	 */
	int srf02_find_object(SRF02_Command measurement_unit);

	int srf02_find_object(SRF02_Command measurement_unit, uint16_t *distance);
	/**
	 * @brief Implements the procedure for changing the I2C address of the SRF02 module as described in the
	 * data sheet.
	 *
	 * @param[in] new_i2c_address the new I2C address of type uint8_t
	 * @return status of the request of type srf02_status_t. SRF02_OPERATION_SUCCESSFUL when the communication with the I2C was succsefful, a status code otherwise
	 */
	srf02_status_t srf02_set_new_i2c_address(uint8_t new_i2c_address);

	/**
	 * @brief Converts a status code of SRF02 functions to an error string.
	 *
	 * Calling free() is not required for the returned value.
	 *
	 * @param[in] srf02_status status code of type srf02_status_t
	 * @return a pointer to a constant string with the appropriate error message
	 */
	const char *srf02_status_to_string(srf02_status_t srf02_status);
};

#endif /* SRF02_H_ */
