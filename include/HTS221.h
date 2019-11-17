 /**
  * @file HTS221.h
  *
  * @brief A header file from libsetila library. It defines HTS221 class for communication with HT221 sensor.
  *
  * @author Goce Boshkovski
  * @date 01 May 2016
  *
  * @copyright GNU General Public License v3
  *
  */

#ifndef HTS221_H_
#define HTS221_H_

#include <stdint.h>

#include "i2c_slave_device.h"

/** \defgroup HTS221_DESC HTS221
 * @ingroup DEV_REG_CMD
 */

/** \defgroup HTS221_REG_DEF HTS221 Register Address Map
 * @ingroup HTS221_DESC
 */
/* @{ */
/** @brief WHO_AM_I register address. */
#define HTS221_WHO_AM_I 0x0F
/** @brief AV_CONF register address. */
#define HTS221_AV_CONF 0x10
/** @brief CTRL_REG1 register address. */
#define HTS221_CTRL_REG1 0x20
/** @brief CTRL_REG2 register address. */
#define HTS221_CTRL_REG2 0x21
/** @brief CTRL_REG3 register address. */
#define HTS221_CTRL_REG3 0x22
/** @brief STATUS_REG register address. */
#define HTS221_STATUS_REG 0x27
/** @brief HUMIDITY_OUT_L register address. */
#define HTS221_HUMIDITY_OUT_L 0x28
/** @brief HUMIDITY_OUT_H register address. */
#define HTS221_HUMIDITY_OUT_H 0x29
/** @brief TEMP_OUT_L register address. */
#define HTS221_TEMP_OUT_L 0x2A
/** @brief TEMP_OUT_H register address. */
#define HTS221_TEMP_OUT_H 0x2B
/** @brief CALIB_0 register address. */
#define HTS221_CALIB_0 0x30
/** @brief CALIB_1 register address. */
#define HTS221_CALIB_1 0x31
/** @brief CALIB_2 register address. */
#define HTS221_CALIB_2 0x32
/** @brief CALIB_3 register address. */
#define HTS221_CALIB_3 0x33
/** @brief CALIB_4 register address. */
#define HTS221_CALIB_4 0x34
/** @brief CALIB_5 register address. */
#define HTS221_CALIB_5 0x35
/** @brief CALIB_6 register address. */
#define HTS221_CALIB_6 0x36
/** @brief CALIB_7 register address. */
#define HTS221_CALIB_7 0x37
/** @brief CALIB_8 register address. */
#define HTS221_CALIB_8 0x38
/** @brief CALIB_9 register address. */
#define HTS221_CALIB_9 0x39
/** @brief CALIB_A register address. */
#define HTS221_CALIB_A 0x3A
/** @brief CALIB_B register address. */
#define HTS221_CALIB_B 0x3B
/** @brief CALIB_C register address. */
#define HTS221_CALIB_C 0x3C
/** @brief CALIB_D register address. */
#define HTS221_CALIB_D 0x3D
/** @brief CALIB_E register address. */
#define HTS221_CALIB_E 0x3E
/** @brief CALIB_F register address. */
#define HTS221_CALIB_F 0x3F
/* @} */

/** \defgroup HTS221_AV_CONF_REG_DESC HTS221 AV_CONF register description
 * @ingroup HTS221_DESC
 */
/* @{ */
/** @brief AV_CONF_AVGT2 bit of AV_CONF register. */
#define HTS221_AV_CONF_AVGT2 0x05
/** @brief AV_CONF_AVGT1 bit of AV_CONF register. */
#define HTS221_AV_CONF_AVGT1 0x04
/** @brief AV_CONF_AVGT0 bit of AV_CONF register. */
#define HTS221_AV_CONF_AVGT0 0x03
/** @brief AV_CONF_AVGH2 bit of AV_CONF register. */
#define HTS221_AV_CONF_AVGH2 0x02
/** @brief AV_CONF_AVGH1 bit of AV_CONF register. */
#define HTS221_AV_CONF_AVGH1 0x01
/** @brief AV_CONF_AVGT0 bit of AV_CONF register. */
#define HTS221_AV_CONF_AVGH0 0x00
/* @} */

/** \defgroup HTS221_CTRL_REG1_REG_DESC HTS221 CTRL_REG1 register description
 * @ingroup HTS221_DESC
 */
/* @{ */
/** @brief PD bit of CTRL_REG1 register. */
#define HTS221_CTRL_REG1_PD 0x07
/** @brief BDU bit of CTRL_REG1 register. */
#define HTS221_CTRL_REG1_BDU 0x02
/** @brief ODR1 bit of CTRL_REG1 register. */
#define HTS221_CTRL_REG1_ODR1 0x01
/** @brief ODR0 bit of CTRL_REG1 register. */
#define HTS221_CTRL_REG1_ODR0 0x00
/* @} */

/** \defgroup HTS221_CTRL_REG2_REG_DESC HTS221 CTRL_REG2 register description
 * @ingroup HTS221_DESC
 */
/* @{ */
/** @brief BOOT bit of CTRL_REG2 register. */
#define HTS221_CTRL_REG2_BOOT 0x07
/** @brief HEATER bit of CTRL_REG2 register. */
#define HTS221_CTRL_REG2_HEATER 0x01
/** @brief ONE_SHOT bit of CTRL_REG2 register. */
#define HTS221_CTRL_REG2_ONE_SHOT 0x00
/* @} */

/** \defgroup HTS221_CTRL_REG3_REG_DESC HTS221 CTRL_REG3 register description
 * @ingroup HTS221_DESC
 */
/* @{ */
/** @brief DRDY_H_L bit of CTRL_REG3 register. */
#define HTS221_CTRL_REG3_DRDY_H_L 0x07
/** @brief PP_OD bit of CTRL_REG3 register. */
#define HTS221_CTRL_REG3_PP_OD 0x06
/** @brief DRDY bit of CTRL_REG3 register. */
#define HTS221_CTRL_REG3_DRDY 0x01
/* @} */

/** \defgroup HTS221_STATUS_REG_REG_DESC HTS221 STATUS_REG register description
 * @ingroup HTS221_DESC
 */
/* @{ */
/** @brief H_DA bit of STATUS_REG register. */
#define HTS221_STATUS_REG_H_DA 0x01
/** @brief T_DA bit of STATUS_REG register. */
#define HTS221_STATUS_REG_T_DA 0x00
/* @} */

/** \defgroup HTS221_DEFAULT_REG_VALUE HTS221 Default values of the registers
 * @ingroup HTS221_DESC
 */
/* @{ */
#define HTS221_AV_CONF_DEFAULT_VALUE 0x1B
#define HTS221_CTRL_REG1_DEFAULT_VALUE 0x00
#define HTS221_CTRL_REG2_DEFAULT_VALUE 0x00
#define HTS221_CTRL_REG3_DEFAULT_VALUE 0x00
#define HTS221_STATUS_REG_DEFAULT_VALUE 0x00
/* @} */

/**
 * \class HTS221
 * @ingroup I2C_SLAVE_DEVICES
 *
 * @brief A class derived from I2CSensor class that describes HTS221 sensor.
 */
class HTS221:public I2C_Slave_Device
{
private:
	float m_temperature_reading = 0.0;					/**< A sensor temperature reading.*/
	float m_humidity_reading = 0.0;						/**< A sensor humidity reading. */
    uint8_t m_calibration_table[16] = { 0x00 };			/**< HTS221 calibration table. */
    uint8_t m_humidity_out[2] = { 0x00 };				/**< A humidity buffer. */
    uint8_t m_temperature_out[2] = { 0x00 };			/**< A temperature buffer. */
    uint8_t CTRL_REG1 = { 0x00 };						/**< Holds CTRL_REG2 value between power ON and OFF requests. */

public:
    HTS221(): I2C_Slave_Device(0x5F) {};
    /**
     * @brief A constructor.
     *
     * Initiate the values of the class members: temperatureReading and humidityReading to values 0.0.
     * The sensor address is set to the value of the sensorAddress argument.
     *
     * @param[in] I2C_slave_address Address of the sensor on the I2C bus.
     */
    HTS221(uint8_t I2C_slave_address): I2C_Slave_Device(I2C_slave_address) {};

	/**
	 * @brief A destructor of the class.
	 */
	~HTS221() {};

	/**
	 * @brief Provides the value of the sensor temperature reading.
	 *
	 * @return float returns the value of the class member temperatureReading.
	 */
	float temperature_reading() const { return m_temperature_reading; };

	/**
	 * @brief Provides the value of the sensor humidity reading.
	 *
	 * @return float returns the value of the class member humidityReading.
	 */
	float humidity_reading() const { return m_humidity_reading; };

	/**
	 * @brief Configures the sensor by setting the values of the main config registers.
	 *
	 * The function sets the content of the registers: AV_CONF and CTRL_REG1
	 * to the desired values.
	 * Before writing the new values to the registers, the function checks if the target is
	 * HTS221 sensor, then reads the calibration table using readSensorCalibrationTable()
	 * and after that proceeds with register changes.
	 *
	 * @param[in] AV_CONF_value the new value of the AV_CONF register
	 * @param[in] CTRL_REG1_value the new value of the CTRL_REG1 register
	 * @return int return 0 for success, ERROR_INIT_HTS221_SENSOR error code in case of a failure
	 */
	int init_sensor(uint8_t AV_CONF_value, uint8_t CTRL_REG1_value);

	/**
	 * @brief This function initializes the sensor with the following configuration:
	 * - block data update bit of CTRL_REG1 set to 1 (inhibit the output register update between the reading of the upper
	 *	 and lower register parts);
	 * - set the output data rate to 12.5Hz;
	 * - pressure and temperature internal average to: AVGT = 32 and AVGP = 64.
	 *
	 * @return int 0 in case of success, _INIT_HTS221_SENSOR error code in case of a failure.
	 */
	int init_sensor(void);


	/**
	 * @brief Configure the sensor for single acquisition of temperature and humidity (ONE_SHOT mode)
	 * with the following settings:
	 * - pressure and temperature internal average values: AVGT = 256 and AVGP = 512;
	 * - block data update bit in CTRL_REG1 set to 1;
	 * - power ON the sensor.
	 *
	 * @return int returns an error code in case of a failure in communication with the sensor, 0 for success.
	 */
	int set_one_shot_mode(void);

	/**
	 * @brief Initiate the measurement process for humidity and temperature.
	 *
	 * The function starts a single acquisition of temperature and humidity by setting the ONE_SHOT bit of CTRL_REG2 register.
	 * After the measuring is done, it reads of the temperature and humidity registers and calculates the
	 * temperature and the humidity values using the private class functions: calculateRealtiveHumidity() and
	 * calculateTemperature().
	 *
	 * @return int returns an error code in case there is a failure in communication with the sensor, 0 for successful measurement cycle.
	 */
	int do_one_shot_measurement(void);

	/**
	 * @brief Reads the content of HTS221 temperature and humidity registers.
	 * After that it calculates the temperature and humidity values.
	 *
	 * This function doesn't work when output data rate is set to ONE_SHOT.
	 *
	 * @return int returns an error code in case there is a failure in communication with the sensor, 0 for successful measurement cycle.
	 */
	int get_sensor_readings(void);

    /**
     * @brief Switch the sensor into a power-down mode by setting PD bit of CTRL_REG1 to 0.
     *
     * @return int an error code in case there is a failure in the communication with the sensor, 0 for success.
     */
    int power_down(void);

    /**
     * @brief Activate the device by setting PD bit of CTRL_REG1 to 1.
     *
     * @return int an error code in case there is a failure in the communication with the sensor, 0 for success.
     */
    int power_up(void);

private:
	/**
	 * @brief Calculates the relative humidity bases on the sensor reading and place the result in the class member "humidityReading".
	 *
	 * @return int always returns 0.
	 */
    int calculate_realtive_humidity();
    /**
     * @brief Calculates the temperature bases on the sensor reading and place the result in the class member "temperatureReading".
     *
     * @return int always returns 0.
     */
    int calculate_temperature();
    /**
     * @brief Reads the content of the calibration table from HTS221 sensor.
     *
     * The start address of the calibration table is HTS221_CALB_0.
     * The MSB bit of the register address is set to 1 for enabling address auto-increment.
     *
     * @return int returns an error code in case there is a failure in communication with the sensor, 0 for successful read.
     */
    int read_calibration_table();
};

#endif /* HTS221_H_ */
