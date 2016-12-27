 /**
  * @file setila_aux.h
  * @brief A header file from libsetila library.
  * It contains the definitions of the error codes used in the library.
  *
  * @author Goce Boshkovski
  * @date 3-Nov-16
  * @copyright GNU General Public License v2.
  *
  */
#ifndef SETILA_ERRORS_H_
#define SETILA_ERRORS_H_

/** \defgroup ERROR_DEF Error codes definitions */

/** \defgroup I2CBUS_ERR Error codes related to I2C bus access
 * @ingroup ERROR_DEF */
/* @{ */
#define ERROR_OPEN_I2C_BUS 						-1
#define ERROR_ATTACH_TO_I2CBUS					-2
#define ERROR_I2C_READ_FAILED					-3
#define ERROR_I2C_WRITE_FAILED					-4
/* @} */

/** \defgroup HTS221_ERR Error codes related to HTS221 sensor
 * @ingroup ERROR_DEF */
/* @{ */
#define ERROR_INIT_HTS221_SENSOR				-10
#define ERROR_READ_HTS221_CALIBRATION_TABLE		-11
#define ERROR_HTS221_MEASUREMENT_FAILED			-12
#define ERROR_NOT_A_HTS221_SENSOR				-13
/* @} */

/** \defgroup LPS25H_ERR Error codes related to LPS25H sensor
 * @ingroup ERROR_DEF */
/* @{ */
#define ERROR_INIT_LPS25H_SENSOR 0x02			-20
#define ERROR_LPS25H_MEASUREMENT_FAILED 0x04	-21
#define ERROR_LPS25H_REGISTRY_WRITE 0x07		-22
#define ERROR_LPS25H_NBR_AVERAGED_SAMPLES		-23
#define ERROR_LPS25H_ENABLE_FIFO_MEAN			-24
#define ERROR_LPS25H_DISABLE_FIFO_MEAN			-25
#define ERROR_LPS25H_SW_RESET					-26
/* @} */

/** \defgroup BMP085_ERR Error codes related to BMP085/180 sensor
 * @ingroup ERROR_DEF */
/* @{ */
#define ERROR_BMP085_START_TEMPERATURE_MEASUREMENT_FAILED	-30
#define ERROR_BMP085_START_PRESSURE_MEASUREMENT_FAILED		-31
#define ERROR_BMP085_READ_TEMPERATURE_FAILED				-32
#define ERROR_BMP085_READ_PRESSURE_FAILED					-33
/* @} */

#endif /* SETILA_ERRORS_H_ */
