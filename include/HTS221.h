/**
 * @file HTS221.h
 *
 * @brief A header file from libsetila library. It defines HTS221 class for
 * communication with HT221 sensor.
 *
 * @author Goce Boshkovski
 * @date 01 May 2016
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef HTS221_H_
#define HTS221_H_

#include <cstdint>

#include "ST_sensor.h"

/**
 * \class HTS221
 * @ingroup I2C_SLAVE_DEVICES
 *
 * @brief A class derived from I2CSensor class that describes HTS221 sensor.
 *
 * @example pi_sense_hat.cpp
 */
class HTS221 : public ST_Sensor // I2C_Slave_Device
{
  static constexpr uint8_t ID{0xBC};

  /** \defgroup HTS221_DESC HTS221
   * @ingroup DEV_REG_CMD
   */

  /** \defgroup HTS221_REG_DEF HTS221 Register Address Map
   * @ingroup HTS221_DESC
   */
  /* @{ */
  static constexpr uint8_t REG_WHO_AM_I{0x0F};
  static constexpr uint8_t REG_AV_CONF{0x10};
  static constexpr uint8_t REG_CTRL_REG1{0x20};
  static constexpr uint8_t REG_CTRL_REG2{0x21};
  static constexpr uint8_t REG_CTRL_REG3{0x22};
  static constexpr uint8_t REG_STATUS_REG{0x27};
  static constexpr uint8_t REG_HUMIDITY_OUT_L{0x28};
  static constexpr uint8_t REG_HUMIDITY_OUT_H{0x29};
  static constexpr uint8_t REG_TEMP_OUT_L{0x2A};
  static constexpr uint8_t REG_TEMP_OUT_H{0x2B};

  static constexpr uint8_t REG_CALIB_0{0x30};
  static constexpr uint8_t H0_rH_x2{
      0}; // Index of the register in calibration table

  static constexpr uint8_t REG_CALIB_1{0x31};
  static constexpr uint8_t H1_rH_x2{
      1}; // Index of the register in calibration table

  static constexpr uint8_t REG_CALIB_2{0x32};
  static constexpr uint8_t T0_degC_x8{
      2}; // Index of the register in calibration table

  static constexpr uint8_t REG_CALIB_3{0x33};
  static constexpr uint8_t T1_degC_x8{
      3}; // Index of the register in calibration table

  static constexpr uint8_t REG_CALIB_4{0x34};

  static constexpr uint8_t REG_CALIB_5{0x35};
  static constexpr uint8_t T1_T0_msb{
      5}; // Index of the register in calibration table

  static constexpr uint8_t REG_CALIB_6{0x36};
  static constexpr uint8_t H0_T0_OUT_L{
      6}; // Index of the register in calibration table

  static constexpr uint8_t REG_CALIB_7{0x37};
  static constexpr uint8_t H0_T0_OUT_H{
      7}; // Index of the register in calibration table

  static constexpr uint8_t REG_CALIB_8{0x38};
  static constexpr uint8_t REG_CALIB_9{0x39};

  static constexpr uint8_t REG_CALIB_A{0x3A};
  static constexpr uint8_t H1_T0_OUT_L{
      10}; // Index of the register in calibration table

  static constexpr uint8_t REG_CALIB_B{0x3B};
  static constexpr uint8_t H1_T0_OUT_H{
      11}; // Index of the register in calibration table

  static constexpr uint8_t REG_CALIB_C{0x3C};
  static constexpr uint8_t T0_OUT_L{
      12}; // Index of the register in calibration table

  static constexpr uint8_t REG_CALIB_D{0x3D};
  static constexpr uint8_t T0_OUT_H{
      13}; // Index of the register in calibration table

  static constexpr uint8_t REG_CALIB_E{0x3E};
  static constexpr uint8_t T1_OUT_L{
      14}; // Index of the register in calibration table

  static constexpr uint8_t REG_CALIB_F{0x3F};
  static constexpr uint8_t T1_OUT_H{
      15}; // Index of the register in calibration table

  /* @} */

  /** \defgroup HTS221_AV_CONF_REG_DESC HTS221 AV_CONF register description
   * @ingroup HTS221_DESC
   */
  /* @{ */
  static constexpr uint8_t AV_CONF_AVGT2_BIT{0x05};
  static constexpr uint8_t AV_CONF_AVGT1_BIT{0x04};
  static constexpr uint8_t AV_CONF_AVGT0_BIT{0x03};
  static constexpr uint8_t AV_CONF_AVGH2_BIT{0x02};
  static constexpr uint8_t AV_CONF_AVGH1_BIT{0x01};
  static constexpr uint8_t AV_CONF_AVGH0_BIT{0x00};
  /* @} */

  /** \defgroup HTS221_CTRL_REG1_REG_DESC HTS221 CTRL_REG1 register description
   * @ingroup HTS221_DESC
   */
  /* @{ */
  static constexpr uint8_t CTRL_REG1_PD_BIT{0x07};
  static constexpr uint8_t CTRL_REG1_BDU_BIT{0x02};
  static constexpr uint8_t CTRL_REG1_ODR1_BIT{0x01};
  static constexpr uint8_t CTRL_REG1_ODR0_BIT{0x00};
  /* @} */

  /** \defgroup HTS221_CTRL_REG2_REG_DESC HTS221 CTRL_REG2 register description
   * @ingroup HTS221_DESC
   */
  /* @{ */
  static constexpr uint8_t CTRL_REG2_BOOT_BIT{0x07};
  static constexpr uint8_t CTRL_REG2_HEATER_BIT{0x01};
  static constexpr uint8_t CTRL_REG2_ONE_SHOT_BIT{0x00};
  /* @} */

  /** \defgroup HTS221_CTRL_REG3_REG_DESC HTS221 CTRL_REG3 register description
   * @ingroup HTS221_DESC
   */
  /* @{ */
  static constexpr uint8_t CTRL_REG3_DRDY_H_L_BIT{0x07};
  static constexpr uint8_t CTRL_REG3_PP_OD_BIT{0x06};
  static constexpr uint8_t CTRL_REG3_DRDY_BIT{0x01};
  /* @} */

  /** \defgroup HTS221_STATUS_REG_REG_DESC HTS221 STATUS_REG register
   * description
   * @ingroup HTS221_DESC
   */
  /* @{ */
  static constexpr uint8_t STATUS_REG_H_DA_BIT{0x01};
  static constexpr uint8_t STATUS_REG_T_DA_BIT{0x00};
  /* @} */

  /** \defgroup HTS221_DEFAULT_REG_VALUE HTS221 Default values of the registers
   * @ingroup HTS221_DESC
   */
  /* @{ */
  static constexpr uint8_t REG_AV_CONF_DEFAULT_VALUE{0x1B};
  static constexpr uint8_t REG_CTRL_REG1_DEFAULT_VALUE{0x00};
  static constexpr uint8_t REG_CTRL_REG2_DEFAULT_VALUE{0x00};
  static constexpr uint8_t REG_CTRL_REG3_DEFAULT_VALUE{0x00};
  static constexpr uint8_t REG_STATUS_REG_DEFAULT_VALUE{0x00};
  /* @} */

private:
  bool m_calibration_table_read =
      false; /**< A flag for reading the calibration table. */
  bool m_device_id_verified =
      false; /**< A flag for verification of the sensor type with the help of
                WHO_AM_I register. */
  float m_temperature_reading = 0.0;        /**< A sensor temperature reading.*/
  float m_humidity_reading = 0.0;           /**< A sensor humidity reading. */
  uint8_t m_calibration_table[16] = {0x00}; /**< HTS221 calibration table. */
  uint8_t m_humidity_out[2] = {0x00};       /**< A humidity buffer. */
  uint8_t m_temperature_out[2] = {0x00};    /**< A temperature buffer. */
  uint8_t m_CTRL_REG1 = {
      0x00}; /**< Holds CTRL_REG1 value between power ON and OFF requests. */
  uint8_t m_CTRL_REG2 = {
      0x00}; /**< Holds CTRL_REG2 value between power ON and OFF requests. */

public:
  HTS221() = delete;

  /**
   *
   * @param[in] interface_type defines the sensor data communication interface
   * (SPI or I2C)
   * @param[in] bus_master_device pointer to the object that represents the
   * master of the SPI/I2C bus where the sensor is connected to
   * @param[in] I2C_slave_address I2C address of the sensor in case the I2C
   * communication
   */
  explicit HTS221(Slave_Device_Type interface_type,
                  Bus_Master_Device *bus_master_device,
                  uint8_t I2C_slave_address)
      : ST_Sensor(interface_type, bus_master_device, I2C_slave_address) {};

  /**
   * @brief A destructor of the class.
   */
  ~HTS221() {}

  virtual int
  set_mode_of_operation(ST_Sensor::MODE_OF_OPERATION mode_of_operation,
                        ST_Sensor::OUTPUT_DATA_RATE output_data_rate =
                            ST_Sensor::OUTPUT_DATA_RATE::ODR_ONE_SHOT) override;

  virtual int set_mode_of_operation(
      ST_Sensor::OUTPUT_DATA_RATE output_data_rate =
          ST_Sensor::OUTPUT_DATA_RATE::ODR_95_Hz,
      ST_Sensor::FULL_SCALE full_scale = ST_Sensor::FULL_SCALE::FS_250_DPS,
      ST_Sensor::MODE_OF_OPERATION mode_of_operation =
          ST_Sensor::MODE_OF_OPERATION::OP_NORMAL_MODE,
      ST_Sensor::FIFO_TYPE fifo_type =
          ST_Sensor::FIFO_TYPE::FIFO_DISABLED) override;

  virtual int set_resolution(uint8_t average_1,
                             uint8_t average_2 = 0x00) override;

  virtual int get_sensor_readings() override;

  /**
   * @brief Provides the value of the sensor temperature reading.
   *
   * @return float returns the value of the class member temperatureReading.
   */
  float temperature_reading() const { return m_temperature_reading; };

  /**
   * @brief Provides the value of the sensor humidity reading.
   *
   * @return float returns the value of the class member humidityReading.
   */
  float humidity_reading() const { return m_humidity_reading; };

private:
  /**
   * @brief Calculates the relative humidity bases on the sensor reading and
   * place the result in the class member ""m_humidity_reading.
   *
   * @return int always returns 0.
   */
  void calculate_realtive_humidity();

  /**
   * @brief Calculates the temperature bases on the sensor reading and place the
   * result in the class member "m_temperature_reading".
   *
   * @return int always returns 0.
   */
  int calculate_temperature();

  /**
   * @brief Reads the content of the calibration table from HTS221 sensor.
   *
   * The start address of the calibration table is HTS221_CALB_0.
   * The MSB bit of the register address is set to 1 for enabling address
   * auto-increment.
   *
   * @return int returns an error code in case there is a failure in
   * communication with the sensor, 0 for successful read.
   */
  int read_calibration_table();

  /**
   *
   * @param output_data_rate
   * @return
   */
  int config_continuous_mode(ST_Sensor::OUTPUT_DATA_RATE output_data_rate);

  /**
   * @brief Switch the sensor into a power-down mode by setting PD bit of
   * CTRL_REG1 to 0.
   *
   * @return int an error code in case there is a failure in the communication
   * with the sensor, 0 for success.
   */
  int power_down(void);

  /**
   * @brief Initiate the measurement process for humidity and temperature.
   *
   * The function starts a single acquisition of temperature and humidity by
   * setting the ONE_SHOT bit of CTRL_REG2 register. After that
   * get_sensor_readings() function is called for reading the measurement
   * outputs.
   *
   * @return int returns an error code in case there is a failure in
   * communication with the sensor; ERROR_SENSOR_READING_TIME_OUT when there is
   * no measurement outputs available SENSOR_READING_WATCHDOG_COUNTER number of
   * reading attempts; 0 for successful measurement cycle.
   */
  int do_one_shot_measurement(void);
};

#endif /* HTS221_H_ */
