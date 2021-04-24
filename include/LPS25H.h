 /**
  * @file LPS25H.h
  *
  * @brief A header file from libsetila library. It defines LPS25H class for communication with LPS25H sensor.
  *
  * @author Goce Boshkovski
  * @date 30 Apr 2016
  *
  * @copyright GNU General Public License v3
  *
  */

#ifndef LPS25H_H_
#define LPS25H_H_

#include "i2c_slave_device.h"

/** @brief Value of the WHO_AM_I register. */
#define LPS25H_ID 0xBD

/** \defgroup LPS25H_DESC LPS25H
 * @ingroup DEV_REG_CMD
 */

/** \defgroup LPS25H_REG_DEF LPS25H Register address map
 * @ingroup LPS25H_DESC
 */
/* @{ */
/** @brief REF_P_XL register address. */
#define LPS25H_REF_P_XL 0x08
/** @brief REF_P_XL register address. */
#define LPS25H_REF_P_L 0x09
/** @brief REF_P_H register address. */
#define LPS25H_REF_P_H 0x0A
/** @brief WHO_AM_I register address. */
#define LPS25H_WHO_AM_I 0x0F
/** @brief RES_CONF register address. */
#define LPS25H_RES_CONF 0x10
/** @brief CTRL_REG1 register address. */
#define LPS25H_CTRL_REG1 0x20
/** @brief CTRL_REG2 register address. */
#define LPS25H_CTRL_REG2 0x21
/** @brief CTRL_REG3 register address. */
#define LPS25H_CTRL_REG3 0x22
/** @brief CTRL_REG4 register address. */
#define LPS25H_CTRL_REG4 0x23
/** @brief INT_CFG register address. */
#define LPS25H_INT_CFG 0x24
/** @brief INT_SOURCE register address. */
#define LPS25H_INT_SOURCE 0x25
/** @brief STATUS_REG register address. */
#define LPS25H_STATUS_REG 0x27
/** @brief POUT_XL register address. */
#define LPS25H_PRESS_POUT_XL 0x28
/** @brief PRESS_OUT_L register address. */
#define LPS25H_PRESS_OUT_L 0x29
/** @brief PRESS_OUT_H register address. */
#define LPS25H_PRESS_OUT_H 0x2A
/** @brief TEMP_OUT_L register address. */
#define LPS25H_TEMP_OUT_L 0x2B
/** @brief TEMP_OUT_H register address. */
#define LPS25H_TEMP_OUT_H 0x2C
/** @brief FIFO_CTRL register address. */
#define LPS25H_FIFO_CTRL 0x2E
/** @brief FIFO_STATUS register address. */
#define LPS25H_FIFO_STATUS 0x2F
/** @brief THS_P_L register address. */
#define LPS25H_THS_P_L 0x30
/** @brief THS_P_H register address. */
#define LPS25H_THS_P_H 0x31
/** @brief RPDS_L register address. */
#define LPS25H_RPDS_L 0x39
/** @brief RPDS_H register address. */
#define LPS25H_RPDS_H 0x3A
/* @} */

/** \defgroup LPS25H_CTRL_REG1_REG_DESC LPS25H CTRL_REG1 register description
  * @ingroup LPS25H_DESC
  */
/* @{ */
/** @brief PD bit of CTRL_REG1 register. */
#define LPS25H_CTRL_REG1_PD 0x07
/** @brief ODR2 bit of CTRL_REG1 register. */
#define LPS25H_CTRL_REG1_ODR2 0x06
/** @brief ODR1 bit of CTRL_REG1 register. */
#define LPS25H_CTRL_REG1_ODR1 0x05
/** @brief ODR0 bit of CTRL_REG1 register. */
#define LPS25H_CTRL_REG1_ODR0 0x04
/** @brief DEFF_EN bit of CTRL_REG1 register. */
#define LPS25H_CTRL_REG1_DEFF_EN 0x03
/** @brief BDU bit of CTRL_REG1 register. */
#define LPS25H_CTRL_REG1_BDU 0x02
/** @brief RESET_AZ bit of CTRL_REG1 register. */
#define LPS25H_CTRL_REG1_RESET_AZ 0x01
/** @brief SIM bit of CTRL_REG1 register. */
#define LPS25H_CTRL_REG1_SIM 0x00
/* @} */

/** \defgroup LPS25H_CTRL_REG2_REG_DESC LPS25H CTRL_REG2 register description
 * @ingroup LPS25H_DESC
 */
/* @{ */
/** @brief BOOT bit of CTRL_REG2 register. */
#define LPS25H_CTRL_REG2_BOOT 0x07
/** @brief FIFO_EN bit of CTRL_REG2 register. */
#define LPS25H_CTRL_REG2_FIFO_EN 0x06
/** @brief WTM_EN bit of CTRL_REG2 register. */
#define LPS25H_CTRL_REG2_WTM_EN 0x05
/** @brief FIFO_MEAN_DEC bit of CTRL_REG2 register. */
#define LPS25H_CTRL_REG2_FIFO_MEAN_DEC 0x04
/** @brief SWRESET bit of CTRL_REG2 register. */
#define LPS25H_CTRL_REG2_SWRESET 0x02
/** @brief ONE_SHOT bit of CTRL_REG2 register. */
#define LPS25H_CTRL_REG2_ONE_SHOT 0x00
/* @} */

/** \defgroup LPS25H_STATUS_REG_REG_DESC LPS25H STATUS_REG register description
  * @ingroup LPS25H_DESC
  */
/* @{ */
/** @brief P_OR bit of STATUS_REG register. */
#define LPS25H_STATUS_REG_P_OR 0x05
/** @brief T_OR bit of STATUS_REG register. */
#define LPS25H_STATUS_REG_T_OR 0x04
/** @brief P_DA bit of STATUS_REG register. */
#define LPS25H_STATUS_REG_P_DA 0x01
/** @brief T_DA bit of STATUS_REG register. */
#define LPS25H_STATUS_REG_T_DA 0x00
/* @} */

/** \defgroup LPS25H_RES_CONF_REG_DESC LPS25H RES_CONF register description
 * @ingroup LPS25H_DESC
 */
/* @{ */
/** @brief AVGT1 bit of RES_CONF register. */
#define LPS25H_RES_CONF_AVGT1 0x03
/** @brief AVGT0 bit of RES_CONF register. */
#define LPS25H_RES_CONF_AVGT0 0x02
/** @brief AVGP1 bit of RES_CONF register. */
#define LPS25H_RES_CONF_AVGP1 0x01
/** @brief AVGP0 bit of RES_CONF register. */
#define LPS25H_RES_CONF_AVGP0 0x00
/* @} */

/** \defgroup LPS25H_FIFO_CTRL_REG_DESC LPS25H FIFO_CTRL register description
 * @ingroup LPS25H_DESC
 */
/* @{ */
/** @brief F_MODE2 bit of FIFO_CTRL register. */
#define FIFO_CTRL_F_MODE2 0x07
/** @brief F_MODE1 bit of FIFO_CTRL register. */
#define FIFO_CTRL_F_MODE1 0x06
/** @brief F_MODE0 bit of FIFO_CTRL register. */
#define FIFO_CTRL_F_MODE0 0x05
/* @} */

/** \defgroup LPS25H_FIFO_WATERMARKS LPS25H FIFO_MEAN_MODE Sample Size
 * @ingroup LPS25H_DESC
 */
/* @{ */
/** @brief WTM_POINT4..0 = 00001 in FIFO_CTRL register. */
#define FIFO_MEAN_MODE_2_SAMPLES 0x01
/** @brief WTM_POINT4..0 = 00011 in FIFO_CTRL register. */
#define FIFO_MEAN_MODE_4_SAMPLES 0x03
/** @brief WTM_POINT4..0 = 00111 in FIFO_CTRL register. */
#define FIFO_MEAN_MODE_8_SAMPLES 0x07
/** @brief WTM_POINT4..0 = 01111 in FIFO_CTRL register. */
#define FIFO_MEAN_MODE_16_SAMPLES 0x0F
/** @brief WTM_POINT4..0 = 11111 in FIFO_CTRL register. */
#define FIFO_MEAN_MODE_32_SAMPLES 0x1F

/** @brief Enumerated values that represents FIFO_MEAN mode moving average size in measurement samples. */
enum class LPS25H_NBR_AVERAGED_SAMPLES {
	AVER_SAMPLES_2,		/**< The pressure output in FIFO_MEAN mode is an average of 2 samples. */
	AVER_SAMPLES_4,		/**< The pressure output in FIFO_MEAN mode is an average of 4 samples. */
	AVER_SAMPLES_8,	 	/**< The pressure output in FIFO_MEAN mode is an average of 8 samples. */
	AVER_SAMPLES_16,	/**< The pressure output in FIFO_MEAN mode is an average of 16 samples. */
	AVER_SAMPLES_32		/**< The pressure output in FIFO_MEAN mode is an average of 32 samples. */
};
/* @} */


/** \class LPS25H
 *  @ingroup I2C_SLAVE_DEVICES
 *
 *  @brief A class derived from I2CSensor class
 *  that describes LPS25H sensor.
 *
 *  @example pi_sense_hat.cpp
 *
 */
class LPS25H: public I2C_Slave_Device
{
private:
	float m_pressure_reading = 0.0;			/**<  The last pressure value measured by the sensor. */
	float m_temperature_reading = 0.0;		/**<  The last temperature value measured by the sensor. */
	uint8_t m_CTRL_REG1 = 0x00;				/**< Holds the value of the CTRL_REG1 between two power ON/Down commands. */

public:
	LPS25H() = default;

	/**
	* @brief A constructor.
	*
	* Initiate the values of the class members:
	* pressureReading is set to 0.0, temperatureReading to 0.0
	* while LPS2H I2C address is set with the value pass to the
	* constructor as an argument. If not given, the address will be
	* set to the default value of 0x01.
	*
	* @param[in] I2C_slave_address an I2C address of the sensor.
	*/
	explicit LPS25H(uint8_t I2C_slave_address):	I2C_Slave_Device(I2C_slave_address)	{};

	/**
	 * @brief A destructor of the class.
	 */
	~LPS25H() {};

	/**
	 * @brief Sensor init functions.
	 *
	 * It configures the sensor behaving by setting the values of the
	 * CTRL_REG1, CTRL_REG2 and RES_CONF registers.
	 * This function must be called before starting the first measurement.
	 *
	 * @param[in] CTRL_REG1_value the new value of the CTRL_REG1 register.
	 * @param[in] CTRL_REG2_value the new value of the CTRL_REG2 register.
	 * @param[in] RES_CONF_value the new value of the RES_CONF register.
	 * @return int return 0 in case of successful init, ERROR_READ/WRITE_FAILED code is case of failure.
	 */
	int init_sensor(uint8_t CTRL_REG1_value, uint8_t CTRL_REG2_value, uint8_t RES_CONF_value);


	/**
	 *
	 * @brief This function sets the following sensor configuration:
	 * - output data rate ODR = 25Hz;
	 * - block data update bit BDU = 1;
	 * - pressure internal average AVGP = 32;
	 * - temperature internal average AVRT = 16;
	 * - FIFO enabled, decimation disabled;
	 * - FIFO mean mode enabled with average on 32 samples.
	 *
	 * @return int, 0 for success, error code in case of a failure.
	 */
	int init_sensor(void);

	/**
	 * @brief Starts one shot measurement of pressure and temperature.
	 *
	 * One shot measurement is started after setting the ONE_SHOT bit in CTRL_REG2 to value 1.
	 * After that get_sensor_readings() is called for getting the measurement outputs.
	 *
     * @return int returns an error code in case there is a failure in communication with the sensor; ERROR_SENSOR_READING_TIME_OUT when there is no measurement outputs
     * available SENSOR_READING_WATCHDOG_COUNTER number of reading attempts; 0 for successful measurement cycle.
     */
	int do_one_shot_measurement();

	/**
	 * @brief Reads the latest humidity and temperature readings from the sensor and calculates the
	 * temperature and humidity values.
	 *
	 * The measurement parameters are set by the init_sensor() function before calling this function.
	 *
     * @return int returns an error code in case there is a failure in communication with the sensor; ERROR_SENSOR_READING_TIME_OUT when there is no measurement outputs
     * available SENSOR_READING_WATCHDOG_COUNTER number of reading attempts; 0 for successful measurement cycle.
     */
	int get_sensor_readings(void);

	/**
	 * @brief Provides the last pressure reading in hPa.
	 *
	 * @return float Pressure value in hPa.
	 */
	float pressure_reading() const { return m_pressure_reading; };


	/**
	 * @brief Provides the last temperature reading in degrees C.
	 *
	 * @return float Temperature value in degrees C.
	 */
	float temperature_reading() const {  return m_temperature_reading; };

	/**
	 * @brief Enable FIFO and select FIFO_MEAN mode.
	 *
	 * The number of averaged samples is provided as an argument.
	 * Other settings (registry values) related to the measurement
	 * parameters like output data rate are set by the initSensor()
	 * functions.
	 *
	 * @param[in] NUM_AVERAGED_SAMPLES number of averaged samples (bits WTM_POINT4-0 of FIFO_CTRL register).
	 * @return int 0 in case the new configuration is written on the sensor, error codes ERROR_READ/WRITE_FAILED
	 * in case of a failure.
	 */
	int enable_FIFO_MEAN(LPS25H_NBR_AVERAGED_SAMPLES NUM_AVERAGED_SAMPLES);

	/**
	 * @brief Disable FIFO mode of operation by reseting the FIFO_EN bit from CTRL_REG2.
	 *
	 * @return int 0 in case of success, error code ERROR_READ/WRITE_FAILED in case of a failure.
	 */
	int disable_FIFO_MEAN(void);

	/**
	 * @brief Turn on the device. The device is in power-down mode when PD = ‘0’ (default value after boot).
	 *
	 * @return 0 in case the new CTRL_REG1 value is sent to the sensor successfully, error code ERROR_WRITE_FAILED in case of a failure.
	 */
	int power_up(void);

	/**
	 * @brief Puts the device is in power-down mode by setting
	 * bit PD of CTRL_REG1 to value 0.
	 *
	 * @return 0 in case the new CTRL_REG1 value is sent to the sensor successfully, error codes ERROR_READ_FAILED/ERROR_WRITE_FAILED
	 * when there is an issue with reading/sending data to the sensor.
	 */
	int power_down(void);

	/**
	 * @brief Configure the sensor for ONE SHOT measurement mode with the following settings:
	 * - pressure and temperature internal average to: AVGT = 16 and AVGP = 32;
	 * - block data update bit of CTRL_REG1 set to 1;
	 * - power ON the device.
	 *
	 * @return int, 0 on success, ERROR_READ/WRITE_FAILED in case of a failure.
	 */
	int set_one_shot_mode(void);

	/**
	 * @brief The device is reset to the power on configuration by setting bits:
	 * SWRESET and BOOT of CTRL_REG1 to ‘1’.
	 *
	 * @return int 0 on success, ERROR_READ/WRITE_FAILED in case there is an error in the communication with the sensor.
	 */
	int SW_reset(void);
};

#endif /* LPS25H_H_ */
