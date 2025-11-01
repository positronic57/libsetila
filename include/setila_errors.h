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

inline constexpr int SENSOR_READING_WATCHDOG_COUNTER{50};

/** \defgroup ERROR_DEF Error Codes */

/** \defgroup BUS_ERR SPI/I2C Communication Error Codes
 * @ingroup ERROR_DEF */
/* @{ */
inline constexpr int ERROR_OPEN_BUS{-1};
inline constexpr int ERROR_NOT_ATTACHED_TO_A_BUS{-2};
inline constexpr int ERROR_ATTACH_TO_BUS{-3};
inline constexpr int ERROR_READ_FAILED{-4};
inline constexpr int ERROR_WRITE_FAILED{-5};
inline constexpr int ERROR_READ_WRTIE_FAILED{-6};
inline constexpr int ERROR_I2C_SELECT_SLAVE{-7};
inline constexpr int ERROR_CONF_BUS_MASTER_FAILED{-8};
inline constexpr int ERROR_UNSUPPORTED_SPI_MODE{-9};
inline constexpr int ERROR_NOT_ATTCHED_TO_BUS{-10};
inline constexpr int ERROR_IO_MODE_NOT_SUPPORTED{-11};
inline constexpr int ERROR_WRONG_DEVICE_MODEL{-12};
inline constexpr int ERROR_UNSUPPORTED_DEVICE_OPTION_CONFIG{-13};
inline constexpr int ERROR_SENSOR_READING_TIME_OUT{-14};
/* @} */

/** \defgroup DEVICE_ERR Device Specific
 * @ingroup ERROR_DEF
 */

/** \defgroup BMP085_ERR BMP085/180 Error Codes
 * @ingroup DEVICE_ERR */
/* @{ */
inline constexpr int ERROR_BMP085_START_TEMPERATURE_MEASUREMENT_FAILED{-30};
inline constexpr int ERROR_BMP085_START_PRESSURE_MEASUREMENT_FAILED{-31};
inline constexpr int ERROR_BMP085_READ_TEMPERATURE_FAILED{-32};
inline constexpr int ERROR_BMP085_READ_PRESSURE_FAILED{-33};
/* @} */

#endif /* SETILA_ERRORS_H_ */
