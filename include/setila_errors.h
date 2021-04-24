 /**
  * @file setila_errors.h
  *
  * @brief Contains definitions of the error codes used in the library.
  *
  * @author Goce Boshkovski
  * @date 3 Nov 2016
  *
  * @copyright GNU General Public License v3
  *
  */

#ifndef SETILA_ERRORS_H_
#define SETILA_ERRORS_H_

#ifndef SENSOR_READING_WATCHDOG_COUNTER
#define SENSOR_READING_WATCHDOG_COUNTER	50
#endif

/** \defgroup ERROR_DEF Error Codes */

/** \defgroup BUS_ERR SPI/I2C Communication Error Codes
 * @ingroup ERROR_DEF */
/* @{ */
#define ERROR_OPEN_BUS							-1
#define ERROR_NOT_ATTACHED_TO_A_BUS				-2
#define ERROR_ATTACH_TO_BUS						-3
#define ERROR_READ_FAILED						-4
#define ERROR_WRITE_FAILED						-5
#define ERROR_READ_WRTIE_FAILED					-6
#define ERROR_I2C_SELECT_SLAVE					-7
#define ERROR_CONF_BUS_MASTER_FAILED			-8
#define ERROR_UNSUPPORTED_SPI_MODE				-9
#define ERROR_NOT_ATTCHED_TO_BUS				-10
#define ERROR_IO_MODE_NOT_SUPPORTED				-11
#define ERROR_WRONG_DEVICE_MODEL				-12
#define ERROR_UNSUPPORTED_DEVICE_OPTION_CONFIG	-13
#define ERROR_SENSOR_READING_TIME_OUT           -14
/* @} */


/** \defgroup DEVICE_ERR Device Specific
* @ingroup ERROR_DEF
*/

/** \defgroup BMP085_ERR BMP085/180 Error Codes
 * @ingroup DEVICE_ERR */
/* @{ */
#define ERROR_BMP085_START_TEMPERATURE_MEASUREMENT_FAILED	-30
#define ERROR_BMP085_START_PRESSURE_MEASUREMENT_FAILED		-31
#define ERROR_BMP085_READ_TEMPERATURE_FAILED				-32
#define ERROR_BMP085_READ_PRESSURE_FAILED					-33
/* @} */


#endif /* SETILA_ERRORS_H_ */
