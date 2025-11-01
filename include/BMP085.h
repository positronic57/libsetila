/**
 * @file BMP085.h
 *
 * @brief A Header file from libsetila library.
 * It defines BMP085 class for communication with BOSCH BMP085 pressure and
 * temperature sensor.
 *
 * @author Goce Boshkovski
 * @date 01 May 2016
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef BMP085_H_
#define BMP085_H_

#include <array>
#include <cstdint>

#include "i2c_slave_device.h"

/** \class BMP085
 *  @ingroup I2C_SLAVE_DEVICES
 *
 *  @brief A class derived from I2CSensor class. It represents BMP085/BMP180
 * pressure and temperature sensor.
 *
 *  @example bmp085_bmp180.cpp
 */
class BMP085 : public I2C_Slave_Device {
public:
  // I2C address of the sensor
  static constexpr uint8_t I2C_ADDRESS{0x77};

  // Addresses of the registers
  static constexpr uint8_t CALIBRATION_TABLE{
      0xAA}; /**< Address of the first register from the calibration table. */
  static constexpr uint8_t DATA_REG_MSB{
      0xF6}; /**< Address of the MSB from the data register. */
  static constexpr uint8_t DATA_REG_LSB{
      0xF7}; /**< Address of the LSB from the data register. */
  static constexpr uint8_t DATA_REG_XLASB{
      0xF8}; /**< Address of the XLASB part from the data register. */
  static constexpr uint8_t CONTROL_REG{
      0xF4}; /**< Address of the control register. */

  // Commands
  static constexpr uint8_t CMD_START_TEMPERATURE_MEASUREMENT{
      0x2E}; /**< Start a temperature measurement. */
  static constexpr uint8_t CMD_START_PRESSURE_MEASUREMENT{
      0x34}; /**< Start a pressure measurement. */

  static constexpr std::array<uint8_t, 4> pressureConversionTimes{
      5,  // Conversion time in ms for ultra low power mode.
      8,  // Conversion time in ms for standard mode.
      14, // Conversion time in ms for high resolution.
      26  // Conversion time in ms for ultra high resolution.
  };

  /**
   * \enum BMP085_Over_Sampling
   *
   * @brief Definition of BMP085/BMP180 modes.
   */
  enum class OverSampling : uint8_t {
    ULTRA_LOW_POWER = 0,  /**< Ultra low power mode. */
    STANDARD,             /**< Standard mode. */
    HIGH_RESOLUTION,      /**< High resolution mode. */
    ULTRA_HIGH_RESOLUTION /**< Ultra high resolution mode. */
  };

private:
  uint8_t m_calibration_table[22] = {0x00}; /**< Calibration table */
  float m_pressure_reading =
      0.00; /**<  The last pressure value measured by the sensor in hPa. */
  float m_temperature_reading =
      0.00; /**<  The last temperature value measured by the sensor. */
  uint8_t m_raw_pressure_data[3] = {
      0x00}; /**<  Buffer for raw pressure data. */
  uint8_t m_raw_temperature_data[2] = {
      0x00}; /**<  Buffer for raw temperature data. */
  OverSampling m_oss{BMP085::OverSampling::STANDARD}; /**< Over sampling mode */

public:
  BMP085() : I2C_Slave_Device(BMP085::I2C_ADDRESS) {};

  /**
   * @brief A constructor.
   *
   * OSS mode is set to standard. Temperature and pressure
   * values as set to 0.0.
   *
   * @param[in] SensorAddress defines the I2C address of the sensor.
   */
  explicit BMP085(uint8_t SensorAddress);

  /**
   * @brief A constructor.
   *
   * I2C address and OSS mode of operation defined via
   * constructor argumnets.Pressure and temperature values are set to 0.0.
   *
   * @param[in] SensorAddress defines the I2C address of the sensor.
   * @param[in] oss defines the sensor mode of operation.
   */
  explicit BMP085(uint8_t SensorAddress, BMP085::OverSampling oss);

  /**
   * @brief A class destructor.
   */
  ~BMP085();

  /**
   * @brief Reads the calibration table from the sensor required for calculating
   * the pressure and temperature values.
   */
  int init_sensor();

  /**
   * @brief Starts the measurement process for temperature and pressure.It
   * calculates the temperature and pressure values.
   */
  int measure_temperature_pressure();

  /**
   * @brief Returns the temperature reading in degrees Celsius.
   *
   * @return float temperature in degrees Celsius
   */
  float temperature_reading() const { return m_temperature_reading; };

  /**
   * @brief Returns the pressure reading in [Pa].
   *
   * @return float pressure reading in [Pa].
   */
  float pressure_reading() const { return m_pressure_reading; };

  /**
   * @brief Sets the value of the oss class member.
   *
   * @param[in] oss the new value for the over sampling mode of the sensor
   */
  void over_sampling(BMP085::OverSampling oss);

private:
  /**
   * @brief Reads the calibration table of the sensor.
   *
   * It is called by the sensor init function.
   *
   * @return int returns 0 in case of success, error code in case of failure.
   */
  int read_calibration_table();
};

#endif /* BMP085_H_ */
