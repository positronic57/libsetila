 /**
  * @file BMP085.h
  * @brief A Header file from libsetila library. It contains the definition of the BMP085 class.
  *
  * @author Goce Boshkovski
  * @date 01-May-16
  * @copyright GNU General Public License v2.
  *
  */

#ifndef BMP085_H_
#define BMP085_H_

#include "I2CSensor.h"

/** \defgroup libMacros Macros defined in the library*/
/* @{ */
/**
 * @def createUword(MSB,LSB)
 * Creates a new unsigned word from two unsigned byte values:
 * \a MSB as most significant byte of the word and \a LSB as least significant byte of the word.
 */
#define createUword(MSB,LSB) (unsigned short)((MSB << 8) | LSB)
/**
 * @def createSword(MSB,LSB)
 * Creates a new signed word from two unsigned byte values:
 * \a MSB as most significant byte of the word and \a LSB as least significant byte of the word.
 */
#define createSword(MSB,LSB) (short)((MSB << 8) | LSB)
/* @} */

/** \defgroup BMP085_REGISTER BMP085 Register address map */
/* @{ */
#define BMP085_CALIBRATION_TABLE 0xAA
#define BMP085_DATA_REG_MSB 0xF6
#define BMP085_DATA_REG_LSB 0xF7
#define BMP085_DATA_REG_XLASB 0xF8
#define BMP085_CONTROL_REG 0xF4
/* @} */

/** \defgroup BMP085_COMMANDS BMP085 Commands */
/* @{ */
#define BMP085_START_TEMPERATURE_MEASUREMENT 0x2E
#define BMP085_START_PRESSURE_MEASUREMENT 0x34
/* @} */

/**
 * \enum BMP085_OverSampling
 *
 * @brief Definition of BMP085/BMP180 modes.
 */
enum BMP085_OverSampling {
							ultraLowPower = 0, /**< Ultra low power mode (oversampling setting = 0). */
							standard,	/**< Standard mode (oversampling setting = 1). */
							highResolution,	/**< High resolution mode (oversampling setting = 2). */
							ultraHighResolution	/**< Ultra high resolution mode (oversampling setting = 3). */
							};

/**
 * BMP085/BMP180 conversion time for each mode of operation.
 */
const long BMP085_PressureConversionTime[]={
		5000000L,	// Conversion time for ultra low power mode.
		8000000L, 	// Conversion time for standard mode.
		14000000L,	// Conversion time for high resolution.
		26000000L	// Conversion time for ultra high resolution.
		};

/** \class BMP085
 *  @brief A class derived from I2CSensor class. It represents BMP085/BMP180 pressure and temperature sensor.
 *
 */
class BMP085: public I2CSensor {
private:
	unsigned char calibrationTable[22]; /**< Calibration table */
	float pressureReading;	/**<  The last pressure value measured by the sensor in hPa. */
	float temperatureReading;	/**<  The last temperature value measured by the sensor. */
	unsigned char rawPressureData[3];	/**<  Buffer for raw pressure data. */
	unsigned char rawTemperatureData[2];	/**<  Buffer for raw temperature data. */
	BMP085_OverSampling oss;	/**< Over sampling mode */

public:
	/**
	 * @brief A constructor.
	 *
	 * OSS mode is set to standard. Temperature and pressure
	 * values as set to 0.0.
	 *
	 * @param[in] SensorAddress defines the I2C address of the sensor.
	 */
	BMP085(unsigned char SensorAddress);

	/**
	 * @brief A constructor.
	 *
	 * I2C address and OSS mode of operation defined via
	 * constructor argumnets.Pressure and temperature values are set to 0.0.
	 *
	 * @param[in] SensorAddress defines the I2C address of the sensor.
	 * @param[in] oss defines the sensor mode of operation.
	 */
	BMP085(unsigned char SensorAddress,BMP085_OverSampling oss);

	/**
	 * @brief A class destructor.
	 */
	~BMP085();

	/**
	 * @brief Reads the calibration table from the sensor required for calculating the pressure and temperature values.
	 */
	int initSensor();

	/**
	 * @brief Starts the measurement process for temperature and pressure.It calculates the temperature and pressure values.
	 */
	int measureTemperaturePressure();

	/**
	 * @brief Returns the temperature reading in degrees Celsius.
	 *
	 * @return float temperature in degrees Celsius
	 */
	float getTemperatureReading();

	/**
	 * @brief Returns the pressure reading in [Pa].
	 *
	 * @return float pressure reading in [Pa].
	 */
	float getPressureReading();

	/**
	 * @brief Sets the value of the oss class member.
	 *
	 * @param[in] oss the new value for the over sampling mode of the sensor
	 */
	void setOversampling(BMP085_OverSampling oss);


private:
	/**
	 * @brief Reads the calibration table of the sensor.
	 *
	 * It is called by the sensor init function.
	 *
	 * @return int returns 0 in case of success, error code in case of failure.
	 */
	int readBMP085CalibrationTable();

};

#endif /* BMP085_H_ */
