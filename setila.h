/*
 * setila.h
 *
 * Created: 20-Feb-16 11:15:40
 * Author: Goce Boshkovski
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

 /**
  * @file setila.h
  * @brief
  * Header file of SETILA library. It contains the prototypes of all
  * functions available in the library, definitions of all macros
  * and constants.
  *
  * @author Goce Boshkovski
  * @date 20-Feb-16
  * @copyright GNU General Public License v2.
  *
  */

#include <stdint.h>

#ifndef _LIBSETILA_H_
#define  _LIBSETILA_H_

/**
 * @brief Sensors present on Sense HAT.
 */
typedef enum eSensorType { HUMIDITY_SENSOR = 1, PRESSURE_SENSOR } TSensorType;

/** \defgroup SENSOR_ADDR_DEF Sense HAT sensor address map */
/* @{ */
#define HUMIDITY_SENSOR_ADDR 0x5F
#define PRESSURE_SENSOR_ADDR 0x05C
/* @} */

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
#define HTS221_CTRL_REG1_ODR1 0x01
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

/** \defgroup ERROR_CODES_DEF Error codes definitions */
/* @{ */
#define ERROR_OPEN_I2C_BUS 0x01
#define ERROR_INIT_HTS221_SENSOR 0x02
#define ERROR_READ_HTS221_CALIBRATION_TABLE 0x03
#define ERROR_HTS221_MEASUREMENT_FAILED 0x04
#define ERROR_LPS25H_MEASUREMRNT_FAILED 0x05
/* @} */

/** \defgroup HTS221_DEFAULT_REG_VALUE Default values of HTS221 registers */
/* @{ */
#define HTS221_AV_CONF_DEFAULT_VALUE 0x1B
#define HTS221_CTRL_REG1_DEFAULT_VALUE 0x00
#define HTS221_CTRL_REG2_DEFAULT_VALUE 0x00
#define HTS221_CTRL_REG3_DEFAULT_VALUE 0x00
#define HTS221_STATUS_REG_DEFAULT_VALUE 0x00
/* @} */

/**
 * @brief Represents Sense HAT sensors readings and configuration.
 */
typedef struct SenseHAT
{
    int fd; /**< File descriptor for accessing Pi I2C device.*/
    uint8_t HTS221CalibrationTable[16]; /**< HTS221 Calibration table. */
    uint8_t HTS221HumidityOut[2];	/**< Buffer for HTS221 humidity registers. */
    uint8_t HTS221TemperatureOut[2]; /**< Buffer for HTS221 temperature registers. */
    float HTS221RelativeHumidityReading; /**< Humidity reading from the HTS221 sensor.*/
    float HTS221TemperatureReading; /**< Temperature reading from the HTS221 sensor.*/
    uint16_t LPS25TemperatureReading; /**< Temperature reading from the LPS25HT sensor.*/
}TSenseHAT;

/** \defgroup SENSE_HAT_FUNC Sense HAT functions */
/* @{ */
/**
 * @brief Init functions. Opens the I2C bus and reads
 * calibration data from the sensors.
 * @param[in,out] pTSenseHAT pointer to the structure that represents the Sense HAT.
 * @param[in] I2Cbus I2C bus device.
 * @return int returns 0 for successful init process, 1 for failure.
 */
int sensehat_initSenseHAT(TSenseHAT *pTSenseHAT,char *I2Cbus);

/**
 * @brief Returns the last temperature value measured by one of the sensors present on the SENSE HAT board.
 * @param[in,out] pTSenseHAT pointer to the structure that represents the Sense HAT.
 * @param[in] fromSensor defines the sensor for measuring temperature.
 * @return float returns 0 the temperature value.
 */
float sensehat_TemperatureReading(TSenseHAT *pTSenseHAT,TSensorType fromSensor);

/**
 * @brief Returns the last relative humidity value in [%] measured by the HTS221 sensor.
 * @param[in,out] pTSenseHAT pointer to the structure that represents the Sense HAT.
 * @return float returns the relative humidity value.
 */
float sensehat_HumidityReading(TSenseHAT *pTSenseHAT);

/**
 * @brief Returns the last pressure value measured by the sensor. 
 * @param[in,out] pTSenseHAT pointer to the structure that represents the Sense HAT.
 * @return float returns the pressure value.
 */
float sensehat_PressureReading(TSenseHAT *pTSenseHAT);

/**
 * @brief Initiates a measurement process for humidity and temperature and
 * fetches the humidity reading from the sensor.
 * @param[in,out] pTSenseHAT pointer to the structure that represents the Sense HAT.
 * @return int returns 0 for successful operation, 1 for failure.
 */
int sensehat_startHumidityMeasurement(TSenseHAT *pTSenseHAT);

/**
 * @brief Calculates the relative humidity value based on the last measurement.
 * @param[in,out] pTSenseHAT pointer to the structure that represents the Sense HAT.
 * @return int returns 0 by deafult.
 */
int sensehat_calculateRealtiveHumidity(TSenseHAT *pTSenseHAT);

/**
 * @brief Calculates the temperature value based on the last measurement done by HTS221 sensor.
 * @param[in,out] pTSenseHAT pointer to the structure that represents the Sense HAT.
 * @return int returns 0 by deafult.
 */
int sensehat_HTS221calculateTemperature(TSenseHAT *pTSenseHAT);

/**
 * @brief Reads the calibration table of HTS221 sensor.
 * @param[in,out] pTSenseHAT pointer to the structure that represents the Sense HAT.
 * @return int returns 0 for successful operation, 1 for failure.
 */
int sensehat_readHumiditySensorCalibrationTable(TSenseHAT *pTSenseHAT);

/**
 * @brief Configures HST221 sensor before use.
 * The function sets the following values for the registers of the sensor:
 * - AV_CONF = 0x1B;
 * - CTRL_REG1 = 0x84;
 * - CTRL_REG2 = 0x00;
 * - CTRL_REG3 = 0x00.
 * @param[in,out] pTSenseHAT pointer to the structure that represents the Sense HAT.
 * @return int returns 0 for successful operation, 1 for failure.
 */
int sensehat_initHTS221Sensor(TSenseHAT *pTSenseHAT);

/**
 * @brief Implements write function of I2C protocol.
 * @param[in] fd file descriptor.
 * @param[in] i2cAddress Address of the device on I2C bus.
 * @param[in] registry Address of the device registry.
 * @param[in] value to be written in the registry.
 * @return int returns 0 for successful operation, 1 for failure.
 */
int i2c_write(int fd,uint8_t i2cAddress,uint8_t registry,uint8_t value);

/**
 * @brief Implements write function of I2C protocol.
 * @param[in] fd file descriptor.
 * @param[in] i2cAddress Address of the device on I2C bus.
 * @param[in] registry Address of the device registry.
 * @param[in] buffer pointer to the buffer that contains the data retrieve from the sensor.
 * @param[in] size buffer size.
 * @return int returns 0 for successful operation, 1 for failure.
 */
int i2c_read(int fd,uint8_t i2cAddress,uint8_t registry,uint8_t *buffer,int size);
/* @} */
#endif
