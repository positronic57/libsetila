/**
 * @file L3GD20.h
 *
 * @brief A header file from libsetila library. It defines L3GD20 class for
 * communication with ST L3GD20 3-axis digital output gyroscope. There is no
 * support for sensor interrupts, the user side must pull for sensor readings.
 * The measurement are always done for all of the axes.
 * There is no interface for selecting which axes will be included/excluded in
 * the measurement. When FIFO is enabled, only FIFO mode is supported.
 *
 * @author Goce Boshkovski
 * @date 29 Sep 2023
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef L3GD20_H_
#define L3GD20_H_

// #include <stdio.h>

#include "ST_sensor.h"

/**
 * \class L3GD20
 *  @ingroup I2C_SLAVE_DEVICES
 *
 *  @brief A class derived from ST_Sensor class
 *  that describes L3GD20 sensor.
 *
 *  @example l3gd20_single_meas_example.cpp
 *  @example l3dg20_FIFO_example.cpp
 */
class L3GD20 : public ST_Sensor {
public:
  /** \defgroup L3GD20_DESC L3GD20
   * @ingroup DEV_REG_CMD
   */

  /** \defgroup L3GD20_FIFO_SRC_REG_DESC L3GD20 FIFO_SRC register description
   * @ingroup L3GD20_DESC
   */
  /* @{ */
  // L3GD20 FIFO status register bits
  static constexpr uint8_t FIFO_SRC_WT_BIT{7};
  static constexpr uint8_t FIFO_SRC_OVRN_BIT{6};
  static constexpr uint8_t FIFO_SRC_EMPTY_BIT{5};
  static constexpr uint8_t FIFO_SRC_FSS4_BIT{4};
  static constexpr uint8_t FIFO_SRC_FSS3_BIT{3};
  static constexpr uint8_t FIFO_SRC_FSS2_BIT{2};
  static constexpr uint8_t FIFO_SRC_FSS1_BIT{1};
  static constexpr uint8_t FIFO_SRC_FSS0_BIT{0};
  /* @} */

  /** \defgroup L3GD20_FIFO_CTRL_REG_DESC L3GD20 FIFO_CTRL register description
   * @ingroup L3GD20_DESC
   */
  /* @{ */
  // L3GD20 FIFO control register
  static constexpr uint8_t FIFO_CTRL_REG_FM2_BIT{7};
  static constexpr uint8_t FIFO_CTRL_REG_FM1_BIT{6};
  static constexpr uint8_t FIFO_CTRL_REG_FM0_BIT{5};
  static constexpr uint8_t FIFO_CTRL_REG_WTM4_BIT{4};
  static constexpr uint8_t FIFO_CTRL_REG_WTM3_BIT{3};
  static constexpr uint8_t FIFO_CTRL_REG_WTM2_BIT{2};
  static constexpr uint8_t FIFO_CTRL_REG_WTM1_BIT{1};
  static constexpr uint8_t FIFO_CTRL_REG_WTM0_BIT{0};
  /* @} */

  /** \defgroup L3GD20_CTRL_REG1_REG_DESC L3GD20 CTRL_REG1 register description
   * @ingroup L3GD20_DESC
   */
  /* @{ */
  // L3GD20 control register 1 bits
  static constexpr uint8_t CTRL_REG1_DR1_BIT{7};
  static constexpr uint8_t CTRL_REG1_DR0_BIT{6};
  static constexpr uint8_t CTRL_REG1_BW1_BIT{5};
  static constexpr uint8_t CTRL_REG1_BW0_BIT{4};
  static constexpr uint8_t CTRL_REG1_PD_BIT{3};
  static constexpr uint8_t CTRL_REG1_Zen_BIT{2};
  static constexpr uint8_t CTRL_REG1_Xen_BIT{1};
  static constexpr uint8_t CTRL_REG1_Yen_BIT{0};
  /* @} */

  /** \defgroup L3GD20_CTRL_REG2_REG_DESC L3GD20 CTRL_REG2 register description
   * @ingroup L3GD20_DESC
   */
  /* @{ */
  // L3GD20 control register bits
  static constexpr uint8_t CTRL_REG2_HPM1_BIT{5};
  static constexpr uint8_t CTRL_REG2_HPM0_BIT{4};
  static constexpr uint8_t CTRL_REG2_HPCF3_BIT{3};
  static constexpr uint8_t CTRL_REG2_HPCF2_BIT{2};
  static constexpr uint8_t CTRL_REG2_HPCF1_BIT{1};
  static constexpr uint8_t CTRL_REG2_HPCF0_BIT{0};
  /* @} */

  /** \defgroup L3GD20_CTRL_REG3_REG_DESC L3GD20 CTRL_REG3 register description
   * @ingroup L3GD20_DESC
   */
  /* @{ */
  // L3GD20 control register bits
  static constexpr uint8_t CTRL_REG3_I1_Int1_BIT{7};
  static constexpr uint8_t CTRL_REG3_I1_Boot_BIT{6};
  static constexpr uint8_t CTRL_REG3_H_Lactive_BIT{5};
  static constexpr uint8_t CTRL_REG3_PP_OD_BIT{4};
  static constexpr uint8_t CTRL_REG3_I2_DRDY_BIT{3};
  static constexpr uint8_t CTRL_REG3_I2_WTM_BIT{2};
  static constexpr uint8_t CTRL_REG3_I2_ORun_BIT{1};
  static constexpr uint8_t CTRL_REG3_I2_Empty_BIT{0};
  /* @} */

  /** \defgroup L3GD20_CTRL_REG4_REG_DESC L3GD20 CTRL_REG4 register description
   * @ingroup L3GD20_DESC
   */
  /* @{ */
  // L3GD20 control register bits
  static constexpr uint8_t CTRL_REG4_BDU_BIT{7};
  static constexpr uint8_t CTRL_REG4_BLE_BIT{6};
  static constexpr uint8_t CTRL_REG4_FS1_BIT{5};
  static constexpr uint8_t CTRL_REG4_FS0_BIT{4};
  static constexpr uint8_t CTRL_REG4_SIM_BIT{0};
  /* @} */

  /** \defgroup L3GD20_CTRL_REG5_REG_DESC L3GD20 CTRL_REG5 register description
   * @ingroup L3GD20_DESC
   */
  /* @{ */
  // L3GD20 control register bits
  static constexpr uint8_t CTRL_REG5_BOOT_BIT{7};
  static constexpr uint8_t CTRL_REG5_FIFO_EN_BIT{6};
  static constexpr uint8_t CTRL_REG5_HPen_BIT{4};
  static constexpr uint8_t CTRL_REG5_INT1_Sel1_BIT{3};
  static constexpr uint8_t CTRL_REG5_INT1_Sel0_BIT{2};
  static constexpr uint8_t CTRL_REG5_Out_Sel1_BIT{1};
  static constexpr uint8_t CTRL_REG5_Out_Sel0_BIT{0};
  /* @} */

  /** \defgroup L3GD20_CTRL_STATUS_REG_DESC L3GD20 STATUS register description
   * @ingroup L3GD20_DESC
   */
  /* @{ */
  // L3GD20 status register bits
  static constexpr uint8_t STATUS_REG_ZYXOR_BIT{7};
  static constexpr uint8_t STATUS_REG_ZOR_BIT{6};
  static constexpr uint8_t STATUS_REG_YOR_BIT{5};
  static constexpr uint8_t STATUS_REG_XOR_BIT{4};
  static constexpr uint8_t STATUS_REG_ZYXDA_BIT{3};
  static constexpr uint8_t STATUS_REG_ZDA_BIT{2};
  static constexpr uint8_t STATUS_REG_YDA_BIT{1};
  static constexpr uint8_t STATUS_REG_XDA_BIT{0};
  /* @} */

  /** \defgroup L3GD20_INT1_CFG_REG_DESC L3GD20 INT1_CGF register description
   * @ingroup L3GD20_DESC
   */
  /* @{ */
  // L3GD20 INT1_CFG register bits
  static constexpr uint8_t INT1_CFG_AND_OR_BIT{7};
  static constexpr uint8_t INT1_CFG_LIR_BIT{6};
  static constexpr uint8_t INT1_CFG_ZHIE_BIT{5};
  static constexpr uint8_t INT1_CFG_ZLIE_BIT{4};
  static constexpr uint8_t INT1_CFG_YHIE_BIT{3};
  static constexpr uint8_t INT1_CFG_YLIE_BIT{2};
  static constexpr uint8_t INT1_CFG_XHIE_BIT{1};
  static constexpr uint8_t INT1_CFG_XLIE_BIT{0};
  /* @} */

  /** \defgroup L3GD20_REG_DEF L3GD20 Register address map
   * @ingroup L3GD20_DESC
   */
  /* @{ */
  static constexpr uint8_t REG_WHO_AM_I{0x0F};
  static constexpr uint8_t REG_CTRL_REG1{0x20};
  static constexpr uint8_t REG_CTRL_REG2{0x21};
  static constexpr uint8_t REG_CTRL_REG3{0x22};
  static constexpr uint8_t REG_CTRL_REG4{0x23};
  static constexpr uint8_t REG_CTRL_REG5{0x24};
  static constexpr uint8_t REG_REFERENCE{0x25};
  static constexpr uint8_t REG_OUT_TEMP{0x26};
  static constexpr uint8_t REG_STATUS_REG{0x27};
  static constexpr uint8_t REG_OUT_X_L{0x28};
  static constexpr uint8_t REG_OUT_X_H{0x29};
  static constexpr uint8_t REG_OUT_Y_L{0x2A};
  static constexpr uint8_t REG_OUT_Y_H{0x2B};
  static constexpr uint8_t REG_OUT_Z_L{0x2C};
  static constexpr uint8_t REG_OUT_Z_H{0x2D};
  static constexpr uint8_t REG_FIFO_CTRL_REG{0x2E};
  static constexpr uint8_t REG_FIFO_SRC_REG{0x2F};
  static constexpr uint8_t REG_INT1_CFG{0x30};
  static constexpr uint8_t REG_INT1_SRC{0x31};
  static constexpr uint8_t REG_INT1_THS_XH{0x32};
  static constexpr uint8_t REG_INT1_THS_XL{0x33};
  static constexpr uint8_t REG_INT1_THS_YH{0x34};
  static constexpr uint8_t REG_INT1_THS_YL{0x35};
  static constexpr uint8_t REG_INT1_THS_ZH{0x36};
  static constexpr uint8_t REG_INT1_THS_ZL{0x37};
  static constexpr uint8_t REG_INT1_DURATION{0x38};
  /* @} */

  /** \defgroup L3GD20_HARDWARE_PARAMS L3GD20 hardware parameters
   * @ingroup L3GD20_DESC
   */
  /* @{ */
  /** ID of the device, stored in WHO_AM_I register */
  static constexpr uint8_t DEVICE_ID{0xD4};
  /** Size of the FIFO buffer in bytes. */
  static constexpr uint8_t FIFO_SIZE_IN_BYTES{192};
  /** Size of the FIFO in 16-bit values */
  static constexpr uint8_t FIFO_SIZE_IN_RAW_VALUES{96};
  /* @} */

public:
  /**
   * \struct Config
   * @brief A container for the parameters of the sensor which defines his
   * behavior and measurement process
   */
  struct Config {
    ST_Sensor::MODE_OF_OPERATION mode_of_operation =
        ST_Sensor::MODE_OF_OPERATION::OP_NORMAL_MODE;
    ST_Sensor::OUTPUT_DATA_RATE output_data_rate =
        ST_Sensor::OUTPUT_DATA_RATE::ODR_95_Hz;
    ST_Sensor::FIFO_TYPE fifo_type = ST_Sensor::FIFO_TYPE::BYPASS_MODE;
    ST_Sensor::FULL_SCALE full_scale = ST_Sensor::FULL_SCALE::FS_250_DPS;
  } config;

  /**
   * \struct Data
   * @brief Holds the raw output values from the sensor for all 3-axes.
   * The array represents the FIFO of the sensor and every third element has the
   * value for the same axis. FIFO[0] = X angular rate from the first
   * measurement FIFO[1] = Y angular rate from the first measurement FIFO[2] = X
   * angular rate from the third measurement FIFO[3] = X angular rate from the
   * second measurement FIFO[4] = Y angular rate from the second measurement
   * FIFO[5] = X angular rate from the second measurement
   *
   * The first 3 elements from the FIFO are the oldest from all measurements
   * present in the FIFO, while the last 3 elements represents the latest sensor
   * outputs.
   *
   * Every call of get_sensor_readings() function will fill this structure with
   * values based on selected measurement method. Even when the FIFO is not in
   * use by the sensor (COnfig.fifo_type = FIFO_DISABLED), the values of a
   * single measurement will end up in the FIFO array at the positions accecible
   * by angular_rate_X, angular_rate_Y and angular_rate_Z members of the
   * structure.
   *
   * The raw sensor outputs are angular rates in milli degrees per second [mdps]
   * per one bit. To convert the raw value in milli degrees per second call
   * angular_rate_in_mdps() function.
   */
  struct Data {
    int16_t FIFO[FIFO_SIZE_IN_RAW_VALUES] =
        {}; /**< A buffer for the FIFO of the sensor */
    const int16_t &angular_rate_X =
        FIFO[FIFO_SIZE_IN_RAW_VALUES -
             3]; /**< Reference for the latest measurement for the X-axis */
    const int16_t &angular_rate_Y =
        FIFO[FIFO_SIZE_IN_RAW_VALUES -
             2]; /**< Reference for the latest measurement for the Y-axis */
    const int16_t &angular_rate_Z =
        FIFO[FIFO_SIZE_IN_RAW_VALUES -
             1]; /**< Reference for the latest measurement for the Z-axis */
  } data;

  /**
   * @brief Converts the raw angular rate value from the sensor to a milli
   * degrees per second based on the configured sensitifity of the sensor.
   *
   * @param[in] conf a reference to the configuration of the sensor
   * @param[in] raw_angular_rate the raw angular rate for one of the axes
   * @return double the angular rate in milli degrees per second
   */
  static double angular_rate_in_mdps(const L3GD20::Config &conf,
                                     int16_t raw_angular_rate);

private:
  bool m_device_id_verified =
      false; /**< A flag for verification of the sensor type with the help of
                WHO_AM_I register. */

  uint8_t m_CTRL_REG1 = 0x07; /**< Holds the last value of the CTRL_REG1. */
  uint8_t m_CTRL_REG2 = 0x00; /**< Holds the last value of the CTRL_REG2. */
  uint8_t m_CTRL_REG3 = 0x00; /**< Holds the last value of the CTRL_REG3. */
  uint8_t m_CTRL_REG4 = 0x00; /**< Holds the last value of the CTRL_REG4. */
  uint8_t m_CTRL_REG5 = 0x00; /**< Holds the last value of the CTRL_REG5. */
  uint8_t m_FIFO_CTRL_REG =
      0x00; /**< Holds the last value of the FIFO_CTRL_REG. */

public:
  L3GD20() = delete;

  /**
   * @brief A constructor.
   *
   * ram[in] interface_type enum for selecting the communication interface of
   * the sensor (I2C or SPI)
   * @param[in] bus_master_device pointer to the object that represents the
   * master of the bus where the sensor is connected
   * @param[in] I2C_slave_address an I2C address of the sensor
   */
  explicit L3GD20(Slave_Device_Type interface_type,
                  Bus_Master_Device *bus_master_device,
                  uint8_t I2C_slave_address)
      : ST_Sensor(interface_type, bus_master_device, I2C_slave_address) {};

  /**
   * @brief A destructor of the class.
   */
  ~L3GD20() {};

  /**
   * @brief Generates new values for the configuration registers based on the
   * parameters in L3GD20::Config structure and sends those values to the
   * sensor. It is a wrapper function for set_mode_of_operation() with four
   * paramters.
   */
  int apply_config();

  virtual int set_mode_of_operation(
      ST_Sensor::OUTPUT_DATA_RATE output_data_rate =
          ST_Sensor::OUTPUT_DATA_RATE::ODR_95_Hz,
      ST_Sensor::FULL_SCALE full_scale = ST_Sensor::FULL_SCALE::FS_250_DPS,
      ST_Sensor::MODE_OF_OPERATION mode_of_operation =
          ST_Sensor::MODE_OF_OPERATION::OP_NORMAL_MODE,
      ST_Sensor::FIFO_TYPE fifo_type =
          ST_Sensor::FIFO_TYPE::FIFO_DISABLED) override;
  virtual int get_sensor_readings() override;
  virtual int set_resolution(uint8_t average_1,
                             uint8_t average_2 = 0x00) override;
  virtual int set_mode_of_operation(
      ST_Sensor::MODE_OF_OPERATION mode_of_operation,
      ST_Sensor::OUTPUT_DATA_RATE output_data_rate =
          ST_Sensor::OUTPUT_DATA_RATE ::ODR_ONE_SHOT) override;

private:
  /**
   * @brief Called by get_sensor_readings() when the sensor is not using FIFO
   * for storing measurements. The sensor readings for all three axes are stored
   * as last elements of Data::FIFO array.
   */
  int get_data_registers();

  int get_data_registers_bypass_mode();
  /**
   * @brief Called by get_sensor_readings() when the FIFO is enabled and the
   * FIFO mode is set to FIFO. The sensor readings for all three axes are stored
   * in Data::FIFO array.
   */
  int get_data_registers_fifo_mode();

  /**
   * @brief Called by get_sensor_readings() when the FIFO is enabled and mode is
   * set to a stream to FIFO mode.
   *
   * //TODO to be implemented
   */
  int get_data_registers_stream_to_fifo_mode();

  /**
   * @brief Called by get_sensor_readings() when the FIFO is enabled and the
   * FIFO mode is set to a stream mode.
   *
   * //TODO to be implemented
   */
  int get_data_registers_stream_mode();

  /**
   * @brief Called by get_sensor_readings() when the FIFO is enabled and the
   * FIFO mode is bypass to FIFO.
   *
   * //TODO to be implemented
   */
  int get_data_registers_bypass_to_fifo_mode();

  /**
   * @brief Aux function used internally for reseting FIFO modes when FIFO is in
   * use.
   */
  inline int reset_FIFO_to_bypass_mode();
};

#endif
