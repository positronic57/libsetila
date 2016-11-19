 /**
  * @file LPS25H.h
  * @brief A Header file from libsetila library. It contains the definition of the LPS25H class.
  *
  * @author Goce Boshkovski
  * @date 30-Apr-16
  * @copyright GNU General Public License v2.
  *
  */

#ifndef LPS25H_H_
#define LPS25H_H_

#include "I2CSensor.h"

/** \defgroup LPS25H_REG_DEF LPS25H Register address map */
/* @{ */
#define LPS25H_REF_P_XL 0x08
#define LPS25H_REF_P_L 0x09
#define LPS25H_REF_P_H 0x0A
#define LPS25H_WHO_AM_I 0x0F
#define LPS25H_RES_CONF 0x10
#define LPS25H_CTRL_REG1 0x20
#define LPS25H_CTRL_REG2 0x21
#define LPS25H_CTRL_REG3 0x22
#define LPS25H_CTRL_REG4 0x23
#define LPS25H_INT_CFG 0x24
#define LPS25H_INT_SOURCE 0x25
#define LPS25H_STATUS_REG 0x27
#define LPS25H_PRESS_POUT_XL 0x28
#define LPS25H_PRESS_OUT_L 0x29
#define LPS25H_PRESS_OUT_H 0x2A
#define LPS25H_TEMP_OUT_L 0x2B
#define LPS25H_TEMP_OUT_H 0x2C
#define LPS25H_FIFO_CTRL 0x2E
#define LPS25H_FIFO_STATUS 0x2F
#define LPS25H_THS_P_L 0x30
#define LPS25H_THS_P_H 0x31
#define LPS25H_RPDS_L 0x39
#define LPS25H_RPDS_H 0x3A
/* @} */

/** \defgroup LPS25H_CTRL_REG1_REG_DESC LPS25H CTRL_REG1 register description */
/* @{ */
#define LPS25H_CTRL_REG1_PD 0x07
#define LPS25H_CTRL_REG1_ODR2 0x06
#define LPS25H_CTRL_REG1_ODR1 0x05
#define LPS25H_CTRL_REG1_ODR0 0x04
#define LPS25H_CTRL_REG1_DEFF_EN 0x03
#define LPS25H_CTRL_REG1_BDU 0x02
#define LPS25H_CTRL_REG1_RESET_AZ 0x01
#define LPS25H_CTRL_REG1_SIM 0x00
/* @} */

/** \defgroup LPS25H_CTRL_REG2_REG_DESC LPS25H CTRL_REG2 register description */
/* @{ */
#define LPS25H_CTRL_REG2_BOOT 0x07
#define LPS25H_CTRL_REG2_FIFO_EN 0x06
#define LPS25H_CTRL_REG2_WTM_EN 0x05
#define LPS25H_CTRL_REG2_FIFO_MEAN_DEC 0x04
#define LPS25H_CTRL_REG2_SWRESET 0x02
#define LPS25H_CTRL_REG2_ONE_SHOT 0x00
/* @} */

/** \defgroup LPS25H_CTRL_REG3_REG_DESC LPS25H CTRL_REG3 register description */
/* @{ */
#define LPS25H_CTRL_REG3_INT_H_L 0x07
#define LPS25H_CTRL_REG3_PP_OD 0x06
#define LPS25H_CTRL_REG3_INT1_S2 0x01
#define LPS25H_CTRL_REG3_INT1_S1 0x00
/* @} */

/** \defgroup LPS25H_CTRL_REG4_REG_DESC LPS25H CTRL_REG4 register description */
/* @{ */
#define LPS25H_CTRL_REG4_P1_EMPTY 0x03
#define LPS25H_CTRL_REG4_P1_WTM 0x02
#define LPS25H_CTRL_REG4_P1_Overrun 0x01
#define LPS25H_CTRL_REG4_P1_DRDY 0x00
/* @} */


/** \defgroup LPS25H_STATUS_REG_REG_DESC LPS25H STATUS_REG register description */
/* @{ */
#define LPS25H_STATUS_P_OR 0x05
#define LPS25H_STATUS_T_OR 0x04
#define LPS25H_STATUS_REG_P_DA 0x01
#define LPS25H_STATUS_REG_T_DA 0x00
/* @} */

/** \defgroup LPS25H_RES_CONF_REG_DESC LPS25H RES_CONF register description */
/* @{ */
#define LPS25H_RES_CONF_AVGT1 0x03
#define LPS25H_RES_CONF_AVGT0 0x02
#define LPS25H_RES_CONF_AVGP1 0x01
#define LPS25H_RES_CONF_AVGP0 0x00
/* @} */

/** \defgroup LPS25H_FIFO_MODES LPS25H FIFO_MODE definitions */
/* @{ */
#define FIFO_MODE_BYPASS 0x00
#define FIFO_MODE_FIFO 0x01
#define FIFO_MODE_STREAM 0x02
#define FIFO_MODE_FIFO_MEAN 0x06
/* @} */

/** \defgroup LPS25H_FIFO_WATERMARKS LPS25H FIFO_MEAN_MODE Sample Size */
/* @{ */
#define FIFO_MEAN_MODE_2_SAMPLES 0x01
#define FIFO_MEAN_MODE_4_SAMPLES 0x03
#define FIFO_MEAN_MODE_8_SAMPLES 0x07
#define FIFO_MEAN_MODE_16_SAMPLES 0x0F
#define FIFO_MEAN_MODE_32_SAMPLES 0x1F
/* @} */

/** \class LPS25H
 *  @brief A class derived from I2CSensor class
 *  that describes LPS25H sensor.
 *
 *  LPS25H is a class that describes LPS25H
 *  pressure and temperature sensor.
 *
 */
class LPS25H: public I2CSensor {
private:
	float pressureReading;	/**<  The last pressure value measured by the sensor. */
	float temperatureReading;	/**<  The last temperature value measured by the sensor. */

public:
	/**
	* @brief A constructor.
	*
	* Initiate the values of the class members:
	* pressureReading is set to 0.0, temperatureReading to 0.0
	* while LPS2H I2C address is set with the value pass to the
	* constructor as an argument. If not given, the address will be
	* set to the default value of 0x01.
	*
	* @param[in] SensorAddress an I2C address of the sensor.
	*/
	LPS25H(unsigned char SensorAddress);
	/**
	 * @brief A destructor of the class.
	 */
	~LPS25H();
	/**
	 * @brief Sensor init functions.
	 *
	 * It configures the sensor behaving by setting the values of the
	 * RES_CONF, CTRL_REG1, CTRL_REG2, CTRL_REG3 registers.
	 * This function must be called before starting the first measurement.
	 *
	 * @param[in] CTRL_REG1 value of the CTRL_REG1 register.
	 * @param[in] CTRL_REG2 value of the CTRL_REG2 register.
	 * @param[in] CTRL_REG3 value of the CTRL_REG3 register.
	 * @param[in] RES_CONF value of the RES_CONF register.
	 * @return int return 0 in case of successful init, ERROR_INIT_LPS25H_SENSOR code is case of failure.
	 */
	int initSensor(unsigned char CTRL_REG1,unsigned char CTRL_REG2,unsigned char CTRL_REG3,unsigned char RES_CONF);
	/**
	 * @brief Starts one shot measurement of pressure and temperature.
	 *
	 * One shot measurement is started after setting the CTRL_REG2 to value 0x01.
	 * After time delay of 5ms, the function reads the values stored in the pressure
	 * and temperature output registers. The time delay is introduced because the
	 * library doesn't support interrupts and the sensor is not configured for generating
	 * interrupt signal. Checking the value of STATUS_REG for end of conversion is replaced
	 * with a time delay to avoid endless loop in case of sensor malfunction.
	 *
	 * After reading the pressure and temperature output registers,
	 * the function calculates the current temperature and pressure values
	 * and updates the pressureReading and temperatureReading members
	 * accordingly.
	 *
	 * @return int error code. 0 for successful attempt, ERROR_LPS25H_MEASUREMENT_FAILED code for failure.
	 */
	int startPressureMeasurement();
	/**
	 * @brief Provides the last pressure reading in hPa.
	 *
	 * @return float Pressure value in hPa.
	 */
	float getPressureReading();
	/**
	 * @brief Provides the last temperature reading in degrees C.
	 *
	 * @return float Temperature value in degrees C.
	 */
	float getTemperaturReading();
};

#endif /* LPS25H_H_ */
