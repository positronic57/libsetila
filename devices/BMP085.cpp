/**
 * @file BMP085.cpp
 *
 * @brief Implementation of BMP085 class.
 *
 * @author Goce Boshkovski
 * @date 27 Apr 2016
 *
 * @copyright GNU General Public License v3
 *
 */

#include <chrono> // for sleep function
#include <thread> // for sleep function

#include "BMP085.h"

#include "setila_aux.h"
#include "setila_errors.h"

BMP085::BMP085(uint8_t i2c_slave_address)
    : I2C_Slave_Device(i2c_slave_address) {}

BMP085::~BMP085() {}

int BMP085::read_calibration_table() {
  return this->read(static_cast<uint8_t>(CALIBRATION_TABLE),
                    m_calibration_table, 22);
}

BMP085::BMP085(uint8_t i2c_slave_address, BMP085::OverSampling oss)
    : I2C_Slave_Device(i2c_slave_address), m_oss(oss) {}

int BMP085::init_sensor() { return read_calibration_table(); }

void BMP085::over_sampling(BMP085::OverSampling oss) { m_oss = oss; }

int BMP085::measure_temperature_pressure() {
  long rawTemperature, temperature = 0;
  long rawPressure = 0, pressure = 0;

  long x1, x2, x3, b3, b5, b6 = 0;
  unsigned long b4, b7 = 0;

  int16_t ac1 =
      aux::createSignedWord(m_calibration_table[0], m_calibration_table[1]);
  int16_t ac2 =
      aux::createSignedWord(m_calibration_table[2], m_calibration_table[3]);
  int16_t ac3 =
      aux::createSignedWord(m_calibration_table[4], m_calibration_table[5]);
  uint16_t ac4 =
      aux::createUnsignedWord(m_calibration_table[6], m_calibration_table[7]);
  uint16_t ac5 =
      aux::createUnsignedWord(m_calibration_table[8], m_calibration_table[9]);
  uint16_t ac6 =
      aux::createUnsignedWord(m_calibration_table[10], m_calibration_table[11]);

  int16_t b1 =
      aux::createSignedWord(m_calibration_table[12], m_calibration_table[13]);
  int16_t b2 =
      aux::createSignedWord(m_calibration_table[14], m_calibration_table[15]);

  int16_t mc =
      aux::createSignedWord(m_calibration_table[18], m_calibration_table[19]);
  int16_t md =
      aux::createSignedWord(m_calibration_table[20], m_calibration_table[21]);

  /* Send the command to start the temperature measurement. */
  if (this->write_byte(CONTROL_REG, CMD_START_TEMPERATURE_MEASUREMENT)) {
    return ERROR_BMP085_START_TEMPERATURE_MEASUREMENT_FAILED;
  }

  /* Wait approximately 4.5ms for temperature conversion. */
  std::this_thread::sleep_for(std::chrono::milliseconds(pressureConversionTimes[static_cast<uint8_t>(
      OverSampling::ULTRA_LOW_POWER)]));

  /* Read the temperature raw value */
  if (this->read(DATA_REG_MSB, m_raw_temperature_data, 2)) {
    return ERROR_BMP085_READ_TEMPERATURE_FAILED;
  }
  rawTemperature =
      (long)((m_raw_temperature_data[0] << 8) + m_raw_temperature_data[1]);

  /* Start the pressure measurement by setting the value of the BMP085 control
   * register.*/
  if (this->write_byte(CONTROL_REG, CMD_START_PRESSURE_MEASUREMENT +
                                        (static_cast<uint8_t>(m_oss) << 6))) {
    return ERROR_BMP085_START_PRESSURE_MEASUREMENT_FAILED;
  }

  /* Wait for the pressure conversion to be done. */
  std::this_thread::sleep_for(std::chrono::milliseconds(pressureConversionTimes[static_cast<uint8_t>(m_oss)]));

  /* Read the pressure raw value. */
  if (this->read(DATA_REG_MSB, m_raw_pressure_data, 2)) {
    return ERROR_BMP085_READ_PRESSURE_FAILED;
  }

  if (this->read(DATA_REG_XLASB, &m_raw_pressure_data[2], 1)) {
    return ERROR_BMP085_READ_PRESSURE_FAILED;
  }

  rawPressure = ((m_raw_pressure_data[0] << 16) +
                 (m_raw_pressure_data[1] << 8) + m_raw_pressure_data[2]) >>
                (8 - static_cast<uint8_t>(m_oss));

  /* Calculate the actual temperature value. */
  x1 = ((rawTemperature - ac6) * ac5) >> 15;
  x2 = (mc << 11) / (x1 + md);
  b5 = x1 + x2;
  temperature = (b5 + 8) >> 4;
  m_temperature_reading = temperature * 0.1;

  /* Calculate the actual pressure value in hPa. */
  b6 = b5 - 4000;
  x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((ac1 * 4 + x3) << static_cast<long>(m_oss)) + 2) / 4;
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;
  b7 = ((unsigned long)rawPressure - b3) *
       (50000 >> static_cast<uint8_t>(m_oss));

  if (b7 < 0x80000000) {
    pressure = (b7 * 2) / b4;
  } else {
    pressure = (b7 / b4) * 2;
  }

  x1 = (pressure >> 8) * (pressure >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * pressure) >> 16;
  pressure += ((x1 + x2 + 3791) >> 4);
  m_pressure_reading = (float)pressure / 100.0;

  return 0;
}
