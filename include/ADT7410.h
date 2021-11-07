/**
 * @file ADT7410.h
 *
 * @brief A Header file from libsetila library.
 * It defines ADT7410 class for communication with Analog Devices ADT7410 temperature sensor.
 *
 * @author Goce Boshkovski
 * @date 1 Nov 2021
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef INCLUDE_ADT7410_H_
#define INCLUDE_ADT7410_H_

#include "i2c_slave_device.h"

/** \defgroup ADT7410_DESC ADT7410
 * @ingroup DEV_REG_CMD
 */

/** \defgroup ADT7410_REG_DEF ADT7410 Register address map
 * @ingroup ADT7410_DESC
 */
/* @{ */

/** @brief The address of the MSB temperature register */
#define ADT7410_TEMPERATURE_REG	0x00

/** @brief The address of status register */
#define ADT7410_STATUS_REG		0x02

/** @brief The address of configuration register */
#define ADT7410_CONFIG_REG		0x03

/** @brief The address of the ID register */
#define ADT7410_ID_REG			0x0B
/* @} */

/** \defgroup ADT7410_REG_DESC ADT7410 registers description
 * @ingroup ADT7410_DESC
 */

/** \defgroup ADT7410_STATUS_REG_DESC ADT7410 status register description
 * @ingroup ADT7410_REG_DESC
 */
/* @{ */
/** @brief Temperature conversion result is available for reading from the temperature registers */
#define ADT7410_STATUS_REG_RDY_BIT 			7
/* @} */

/** \defgroup ADT7410_CONFIGURATION_REG_DESC ADT7410 configuration register description
 * @ingroup ADT7410_REG_DESC
 */
/* @{ */
/** @brief This bit sets up the resolution of the ADC when converting */
#define ADT7410_CONFIG_REG_RESOLUTION_BIT	7

/** @brief Second operational mode bit */
#define ADT7410_CONFIG_REG_OP_BIT1			5

/** @brief First operational mode bit */
#define ADT7410_CONFIG_REG_OP_BIT2			6
/* @} */


/**
 * \class ADT7410
 *
 * @ingroup I2C_SLAVE_DEVICES
 *
 * @brief Class which represents ADT7410 sensor with I2C interface
 *
 * @example adt7410_example.cpp
 */
class ADT7410: public I2C_Slave_Device
{
public:
	/**
	 * \enum I2C_ADDRESS
	 *
	 * @brief List of I2C addresses which can be assigned to the sensor using the address pins: A0 and A1.
	 */
	enum class I2C_ADDRESS:uint8_t {
		_0x48 = 0x48,		/**< 0x48 - A1 low, A0 low */
		_0x49,       		/**< 0x49 - A1 low, A1 high */
		_0x4A,       		/**< 0x4A - A1 high, A0 low */
		_0x4B        		/**< 0x4B - A1 high, A0 high */
	};

	/**
	 * \enum MODE_OF_OPERATION
	 *
	 * @brief List of available operational modes
	 */
	enum class MODE_OF_OPERATION:int {
		CONTINUOUS_CONVERSION = 0,	/**< Continuous conversion. When one conversion is finished, the ADT7410 starts another */
	    ONE_SHOT,              		/**< The conversion is done once after the appropriate value is written in the configuration register */
		ONE_SAMPLE_PER_SECOND,    	/**< Make one temperature sample per second  */
		SHUTDOWN                  	/**< All circuitry except interface circuitry is powered down */
	};

	/**
	 * \enum RESOLUTION
	 *
	 * @brief The resolution of the ADC for temperature conversion in bits.
	 */
	enum class RESOLUTION:int {
		RES_0_0625 = 0,				/**< 13-bit resolution. Sign bit + 12 bits gives a temperature resolution of 0.0625°C */
		RES_0_0078     				/**< 16-bit resolution. Sign bit + 15 bits gives a temperature resolution of 0.0078°C */
	};

private:
	float m_temperature = 0.0;			/**< Temperature reading in degree Celsius*/
	uint8_t m_config_register = 0x00;	/**< The last value of the control register sent to the sensor */
	ADT7410::RESOLUTION m_resolution = ADT7410::RESOLUTION::RES_0_0625;	/**< The last value for the resolution of the ADC when converting set in the sensor */

	ADT7410() = default;

public:
	/**
	 * @brief Constructor.
	 *
	 * @param I2C_address One of the I2C addresses assigned to ADT7410
	 */
	ADT7410(uint8_t I2C_address) : I2C_Slave_Device(I2C_address) {};

	~ADT7410() {};

	/**
	 * @brief Sets the operational mode of the ADT7410 and the resolution of the internal ADC
	 *
	 * The function builds the corresponding control register value based on the given operational mode and resolution.
	 *
	 * @param mode the new operational mode of the sensor
	 * @param resolution the new value for the resolution of the ADC when converting
	 *
	 * @return 0 in case of success, ERROR_WRITE_FAILED in case there is an issue in communication with the sensor
	 */
	int set_mode_of_operation(const ADT7410::MODE_OF_OPERATION mode, const ADT7410::RESOLUTION resolution = ADT7410::RESOLUTION::RES_0_0625);

	/**
	 * @brief Sets the resolution of the internal ADC.
	 *
	 * It updates the control register of the sensor based on the selected resolution.
	 *
	 * @param resolution enum value which defines the 13 or 16 bit resolution
	 *
	 * @return 0 in case of success, ERROR_WRITE_FAILED in case there is an issue in communication with the sensor
	 */
	int set_resolution(const ADT7410::RESOLUTION resolution);

	/**
	 * @brief Reads the temperature value from the sensor.
	 *
	 * The status register value is used to check if the temperature registers contains a valid data.
	 * The register is read in a loop till the bit RDY is set to 0 or the maximal number of read attempts are reached.
	 *
	 * @return 0 in case of successful data transfer, ERROR_READ_FAILED in case of issue in data read over I2C,
	 * or ERROR_SENSOR_READING_TIME_OUT in case the temperature conversion is not finished in the predefined period of time to avoid blocking.
	 */
	int get_sensor_readings();

	/**
	 * @brief Auxiliary function for switching the device in the shut down mode. I2C interface stays active.
	 * A wrapper function for the set_mode_of_operation().
	 *
	 * @return 0 in case of success, ERROR_WRITE_FAILED in case there is an issue in communication with the sensor
	 */
	int shutdown() { return set_mode_of_operation(ADT7410::MODE_OF_OPERATION::SHUTDOWN); };

	/**
	 * @brief Auxiliary function for bringing the device back from the shut down mode by enabling the continuous conversion operational mode.
	 *
	 * A wrapper function for the set_mode_of_operation().
	 *
	 * @return 0 in case of success, ERROR_WRITE_FAILED in case there is an issue in communication with the sensor
	 */
	int power_up() { return set_mode_of_operation(ADT7410::MODE_OF_OPERATION::CONTINUOUS_CONVERSION); };

	/**
	 * @brief Access to the class member that holds the last temperature reading in degree Celsius.
	 *
	 * @return float temperature reading in degree Celsius
	 */
	float temperature() const { return m_temperature; };

};

#endif /* INCLUDE_ADT7410_H_ */
