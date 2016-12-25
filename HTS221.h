 /**
  * @file HTS221.h
  * @brief A header file from libsetila library. It contains the definition of the HTS221 class.
  *
  * @author Goce Boshkovski
  * @date 01-May-16
  * @copyright GNU General Public License v2.
  *
  */

#ifndef HTS221_H_
#define HTS221_H_

#include <cinttypes>
#include "I2CSensor.h"

/** \defgroup HTS221_REG_DEF HTS221 Register address map */
/* @{ */
#define HTS221_WHO_AM_I 0x0F
#define HTS221_AV_CONF 0x10
#define HTS221_CTRL_REG1 0x20
#define HTS221_CTRL_REG2 0x21
#define HTS221_CTRL_REG3 0x22
#define HTS221_STATUS_REG 0x27
#define HTS221_HUMIDITY_OUT_L 0x28
#define HTS221_HUMIDITY_OUT_H 0x29
#define HTS221_TEMP_OUT_L 0x2A
#define HTS221_TEMP_OUT_H 0x2B
#define HTS221_CALIB_0 0x30
#define HTS221_CALIB_1 0x31
#define HTS221_CALIB_2 0x32
#define HTS221_CALIB_3 0x33
#define HTS221_CALIB_4 0x34
#define HTS221_CALIB_5 0x35
#define HTS221_CALIB_6 0x36
#define HTS221_CALIB_7 0x37
#define HTS221_CALIB_8 0x38
#define HTS221_CALIB_9 0x39
#define HTS221_CALIB_A 0x3A
#define HTS221_CALIB_B 0x3B
#define HTS221_CALIB_C 0x3C
#define HTS221_CALIB_D 0x3D
#define HTS221_CALIB_E 0x3E
#define HTS221_CALIB_F 0x3F
/* @} */

/** \defgroup HTS221_AV_CONF_REG_DESC HTS221 AV_CONF Register description */
/* @{ */
#define HTS221_AV_CONF_AVGT2 0x05
#define HTS221_AV_CONF_AVGT1 0x04
#define HTS221_AV_CONF_AVGT0 0x03
#define HTS221_AV_CONF_AVGH2 0x02
#define HTS221_AV_CONF_AVGH1 0x01
#define HTS221_AV_CONF_AVGH0 0x00
/* @} */

/** \defgroup HTS221_CTRL_REG1_REG_DESC HTS221 CTRL_REG1 register description */
/* @{ */
#define HTS221_CTRL_REG1_PD 0x07
#define HTS221_CTRL_REG1_BDU 0x02
#define HTS221_CTRL_REGint 1_ODR1 0x01
#define HTS221_CTRL_REG1_ODR0 0x00
/* @} */

/** \defgroup HTS221_CTRL_REG2_REG_DESC HTS221 CTRL_REG2 register description */
/* @{ */
#define HTS221_CTRL_REG2_BOOT 0x07
#define HTS221_CTRL_REG2_HEATER 0x01
#define HTS221_CTRL_REG2_ONE_SHOT 0x00
/* @} */

/** \defgroup HTS221_CTRL_REG3_REG_DESC HTS221 CTRL_REG3 register description */
/* @{ */
#define HTS221_CTRL_REG3_DRDY_H_L 0x07
#define HTS221_CTRL_REG3_PP_OD 0x06
#define HTS221_CTRL_REG3_DRDY 0x01
/* @} */

/** \defgroup HTS221_STATUS_REG_REG_DESC HTS221 STATUS_REG register description */
/* @{ */
#define HTS221_STATUS_REG_H_DA 0x01
#define HTS221_STATUS_REG_T_DA 0x00
/* @} */

/** \defgroup HTS221_DEFAULT_REG_VALUE HTS221 Default values of the registers */
/* @{ */
#define HTS221_AV_CONF_DEFAULT_VALUE 0x1B
#define HTS221_CTRL_REG1_DEFAULT_VALUE 0x00
#define HTS221_CTRL_REG2_DEFAULT_VALUE 0x00
#define HTS221_CTRL_REG3_DEFAULT_VALUE 0x00
#define HTS221_STATUS_REG_DEFAULT_VALUE 0x00
/* @} */

/**
 * \class HTS221
 *
 * @brief A class derived from I2CSensor class that describes a HTS221 sensor.
 */
class HTS221:public I2CSensor{
private:
	float temperatureReading;					/**< A sensor temperature reading.*/
	float humidityReading;						/**< A sensor humidity reading. */
    unsigned char HTS221CalibrationTable[16];	/**< HTS221 calibration table. */
    unsigned char HTS221HumidityOut[2];			/**< A humidity buffer. */
    unsigned char HTS221TemperatureOut[2];		/**< A temperature buffer. */
    unsigned char CTRL_REG1;					/**< Holds CTRL_REG2 value between power ON and OFF requests. */

public:
    /**
     * @brief A constructor.
     *
     * Initiate the values of the class members: temperatureReading and humidityReading to values 0.0.
     * The sensor address is set to the value of the sensorAddress argument.
     *
     * @param[in] sensorAddress Address of the sensor on the I2C bus.
     */
    HTS221(unsigned char sensorAddress);

	/**
	 * @brief A destructor of the class.
	 */
	~HTS221();

	/**
	 * @brief Provides the value of the sensor temperature reading.
	 *
	 * @return float returns the value of the class member temperatureReading.
	 */
	float TemperatureReading();

	/**
	 * @brief Provides the value of the sensor humidity reading.
	 *
	 * @return float returns the value of the class member humidityReading.
	 */
	float HumidityReading();

	/**
	 * @brief Configures the sensor by setting the values of the main config registers.
	 *
	 * The function sets the content of the following registers to the desired values:
	 * AV_CONF, CTRL_REG1, CTRL_REG2 and CTR_REG3.
	 * Before writing the new values to the registers, the function checks if the target is
	 * HTS221 sensor, then reads the calibration table using readSensorCalibrationTable()
	 * and after that proceeds with register changes.
	 *
	 * @param[in] AV_CONF_value the new value of the AV_CONF register
	 * @param[in] CTRL_REG1_value the new value of the CTRL_REG1 register
	 * @param[in] CTRL_REG2_value the new value of the CTRL_REG2 register
	 * @param[in] CTRL_REG3_value the new value of the CTRL_REG3 register
	 * @return int return 0 for success, ERROR_INIT_HTS221_SENSOR error code in case of a failure
	 */
	int initSensor(unsigned char AV_CONF_value,unsigned char CTRL_REG1_value,unsigned char CTRL_REG2_value,unsigned char CTRL_REG3_value);

	/**
	 * @brief Configure the sensor for single acquisition of temperature and humidity (ONE_SHOT mode).
	 *
	 * @return int returns an error code in case of a failure in communication with the sensor, 0 for success.
	 */
	int setOneShotMode(void);

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
	int doOneShotMeasurement(void);

	/**
	 * @brief Reads the content of HTS221 temperature and humidity registers.
	 * After that it calculates the temperature and humidity values.
	 *
	 * This function doesn't work when output data rate is set to ONE_SHOT.
	 *
	 * @return int returns an error code in case there is a failure in communication with the sensor, 0 for successful measurement cycle.
	 */
	int getSensorReadings(void);

private:
	/**
	 * @brief Calculates the relative humidity bases on the sensor reading and place the result in the class member "humidityReading".
	 *
	 * @return int always returns 0.
	 */
    int calculateRealtiveHumidity();
    /**
     * @brief Calculates the temperature bases on the sensor reading and place the result in the class member "temperatureReading".
     *
     * @return int alaways returns 0.
     */
    int calculateTemperature();
    /**
     * @brief Reads the content of the calibration table from HTS221 sensor.
     *
     * The start address of the calibration table is HTS221_CALB_0.
     * The MSB bit of the register address is set to 1 for enabling address auto-increment.
     *
     * @return int returns an error code in case there is a failure in communication with the sensor, 0 for successful read.
     */
    int readSensorCalibrationTable();

    /**
     * @brief Switch the sensor into a power-down mode by setting PD bit of CTRL_REG1 to 0.
     *
     * @return int an error code in case there is a failure in the communication with the sensor, 0 for success.
     */
    int powerDown(void);

    /**
     * @brief Activate the device by setting PD bit of CTRL_REG1 to 1.
     *
     * @return int an error code in case there is a failure in the communication with the sensor, 0 for success.
     */
    int powerUp(void);

};

#endif /* HTS221_H_ */
