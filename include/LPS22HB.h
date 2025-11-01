/**
 * @file LPS22HB.h
 *
 * @brief A header file from libsetila library. It defines LPS22HB class for
 * communication with LPS22HB sensor.
 *
 * @author Goce Boshkovski
 * @date 17.04.2021
 *
 * @copyright GNU General Public License v3
 *
 * Supported LPS22HB modes of operation:
 * - ONE SHOT.
 *
 * Only a measurement of absolute temperature and pressure values is supported.
 *
 * Typical use case:
 *
 * 1. Call the constructor with LPS22HB I2C address, communication interface
 * type and pointer to the bus master object. For example 0x5C:
 *
 *  LPS22HB(Slave_Device_Type::I2C_SLAVE_DEVICE, i2c_bus_master, 0x5C);
 *
 * 2. Assign ONE SHOT as mode of operation
 *
 *  set_mode_of_operation(ST_Sensor::mode_of_operation_t::OP_ONE_SHOT);
 *
 * 3. Perform a measurement and get the pressure and temperature readings
 *
 *  get_sensor_readings();
 *
 * @example arduino_mkr_env_shield_rev2.cpp
 *
 */

#ifndef LPS22HB_H_
#define LPS22HB_H_

#include "ST_sensor.h"

/**
 * \class LPS22HB
 *  @ingroup I2C_SLAVE_DEVICES
 *
 *  @brief A class derived from I2CSensor class
 *  that describes LPS22HB sensor.
 *
 *  @example arduino_mkr_env_shield_rev2.cpp
 */
class LPS22HB : public ST_Sensor {
public:
  /** @brief Value of the WHO_AM_I register. */
  static constexpr uint8_t ID{0xB1};

  /** @brief Value for enabling the power down/single shot mode of operation. */
  static constexpr uint8_t POWER_DOWN_MODE_DEF{0x0F};

  /** @brief PLS22HB I2C address when SA0 pad is contected to ground. */
  static constexpr uint8_t I2C_ADDR_SA0_0{0x5C};

  /** @brief PLS22HB I2C address when SA0 pad is connected to voltage supply. */
  static constexpr uint8_t I2C_ADDR_SA0_1{0x5D};

  /** \defgroup DESC LPS22HB
   * @ingroup DEV_REG_CMD
   */

  /** \defgroup REG_DEF LPS22HB Register address map
   * @ingroup DESC
   */
  /* @{ */
  static constexpr uint8_t REG_INT_CFG{0x0B};
  static constexpr uint8_t REG_THS_P_L{0x0C};
  static constexpr uint8_t REG_THS_P_H{0x0D};
  static constexpr uint8_t REG_WHO_AM_I{0x0F};
  static constexpr uint8_t REG_CTRL_REG1{0x10};
  static constexpr uint8_t REG_CTRL_REG2{0x11};
  static constexpr uint8_t REG_CTRL_REG3{0x12};
  static constexpr uint8_t REG_FIFO_CTRL{0x14};
  static constexpr uint8_t REG_REF_P_XL{0x15};
  static constexpr uint8_t REG_REF_P_L{0x16};
  static constexpr uint8_t REG_REF_P_H{0x17};
  static constexpr uint8_t REG_RPDS_L{0x18};
  static constexpr uint8_t REG_RPDS_H{0x19};
  static constexpr uint8_t REG_RES_CONF{0x1A};
  static constexpr uint8_t REG_INT_SOURCE{0x25};
  static constexpr uint8_t REG_FIFO_STATUS{0x26};
  static constexpr uint8_t REG_STATUS_REG{0x27};
  static constexpr uint8_t REG_PRESS_OUT_XL{0x28};
  static constexpr uint8_t REG_PRESS_OUT_L{0x29};
  static constexpr uint8_t REG_PRESS_OUT_H{0x2A};
  static constexpr uint8_t REG_TEMP_OUT_L{0x2B};
  static constexpr uint8_t REG_TEMP_OUT_H{0x2C};
  static constexpr uint8_t REG_LPFP_RES{0x33};
  /* @} */

  /** \defgroup CTRL_REG1_REG_DESC LPS22HB CTRL_REG1 register
   * description
   * @ingroup DESC
   */
  /* @{ */
  static constexpr uint8_t CTRL_REG1_ODR2_BIT{0x06};
  static constexpr uint8_t CTRL_REG1_ODR1_BIT{0x05};
  static constexpr uint8_t CTRL_REG1_ODR0_BIT{0x04};
  static constexpr uint8_t CTRL_REG1_EN_LPFP_BIT{0x03};
  static constexpr uint8_t CTRL_REG1_LPFP_CFG_BIT{0x02};
  static constexpr uint8_t CTRL_REG1_BDU_BIT{0x01};
  static constexpr uint8_t CTRL_REG1_SIM_BIT{0x00};
  /* @} */

  /** \defgroup CTRL_REG2_REG_DESC LPS22HB CTRL_REG2 register
   * description
   * @ingroup DESC
   */
  /* @{ */
  static constexpr uint8_t CTRL_REG2_BOOT_BIT{0x07};
  static constexpr uint8_t CTRL_REG2_FIFO_EN_BIT{0x06};
  static constexpr uint8_t CTRL_REG2_STOP_ON_FTH_BIT{0x05};
  static constexpr uint8_t CTRL_REG2_IF_ADD_INC_BIT{0x04};
  static constexpr uint8_t CTRL_REG2_I2C_DIS_BIT{0x03};
  static constexpr uint8_t CTRL_REG2_SWRESET_BIT{0x02};
  static constexpr uint8_t CTRL_REG2_ONE_SHOT_BIT{0x00};
  /* @} */

  /** \defgroup CTRL_REG3_REG_DESC LPS22HB CTRL_REG3 register
   * description
   * @ingroup DESC
   */
  /* @{ */
  static constexpr uint8_t CTRL_REG3_INT_H_L_BIT{0x07};
  static constexpr uint8_t CTRL_REG3_PP_OD_BIT{0x06};
  static constexpr uint8_t CTRL_REG3_F_FSS5_BIT{0x05};
  static constexpr uint8_t CTRL_REG3_F_FTH_BIT{0x04};
  static constexpr uint8_t CTRL_REG3_F_OVR_BIT{0x03};
  static constexpr uint8_t CTRL_REG3_DRDY_BIT{0x02};
  static constexpr uint8_t CTRL_REG3_INT_S2_BIT{0x01};
  static constexpr uint8_t CTRL_REG3_INT_S1_BIT{0x00};
  /* @} */

  /** \defgroup STATUS_REG_REG_DESC LPS22HB STATUS_REG register
   * description
   * @ingroup DESC
   */
  /* @{ */
  /** @brief Temperature data overrun T_OR bit of STATUS_REG register. */
  static constexpr uint8_t STATUS_REG_T_OR_BIT{0x05};
  static constexpr uint8_t STATUS_REG_P_OR_BIT{0x04};
  static constexpr uint8_t STATUS_REG_T_DA_BIT{0x01};
  static constexpr uint8_t PS22HB_STATUS_REG_P_DA_BIT{0x00};
  /* @} */

  /** \defgroup RES_CONF_REG_DESC LPS22HB RES_CONF register
   * description
   * @ingroup DESC
   */
  /* @{ */
  static constexpr uint8_t RES_CONF_LC_EN_BIT{0x00};
  /* @} */

  /** \defgroup FIFO_CTRL_REG_DESC LPS22HB FIFO_CTRL register
   * description
   * @ingroup DESC
   */
  /* @{ */
  static constexpr uint8_t FIFO_CTRL_F_MODE2_BIT{0x07};
  static constexpr uint8_t FIFO_CTRL_F_MODE1_BIT{0x06};
  static constexpr uint8_t FIFO_CTRL_F_MODE0_BIT{0x05};
  static constexpr uint8_t FIFO_CTRL_WTM4_BIT{0x04};
  static constexpr uint8_t FIFO_CTRL_WTM3_BIT{0x03};
  static constexpr uint8_t FIFO_CTRL_WTM2_BIT{0x02};
  static constexpr uint8_t FIFO_CTRL_WTM1_BIT{0x01};
  static constexpr uint8_t FIFO_CTRL_WTMO_BIT{0x00};
  /* @} */

  /* @} */

private:
  bool m_device_id_verified =
      false; /**< A flag for verification of the sensor type with
                the help of WHO_AM_I register. */
  float m_pressure_reading =
      0.0; /**<  The last pressure value measured by the sensor. */
  float m_temperature_reading =
      0.0; /**<  The last temperature value measured by the sensor. */
  uint8_t m_CTRL_REG1 = 0x00; /**< Holds the last value of the CTRL_REG1. */
  uint8_t m_CTRL_REG2 = 0x10; /**< Holds the last value of the CTRL_REG2. */
  uint8_t m_CTRL_REG3 = 0x00; /**< Holds the last value of the CTRL_REG3. */

  LPS22HB() = delete;

public:
  /**
   * @brief A constructor.
   *
   * @param[in] interface_type enum for selecting the communication interface of
   * the sensor (I2C or SPI)
   * @param[in] bus_master_device pointer to the object that represents the
   * master of the bus where the sensor is connected
   * @param[in] I2C_slave_address an I2C address of the sensor
   */
  explicit LPS22HB(Slave_Device_Type interface_type,
                   Bus_Master_Device *bus_master_device,
                   uint8_t I2C_slave_address)
      : ST_Sensor(interface_type, bus_master_device, I2C_slave_address) {};

  /**
   * @brief A destructor of the class.
   */
  ~LPS22HB() {};

  /**
   * @brief A low level function for custom sensor configuration using the
   * control registers of the sensor.
   *
   * Can be used when the higher level function like set_mode_of_operation() is
   * not enough for the application. It will change the values of the CTRL_REG1,
   * CTRL_REG2 and CTRL_REG3 registers.
   *
   * @param[in] CTRL_REG1_value the new value of the CTRL_REG1 register.
   * @param[in] CTRL_REG2_value the new value of the CTRL_REG2 register.
   * @param[in] CTRL_REG3_value the new value of the CTRL_REG3 register.
   *
   * @return int returns 0 in case of successful init, ERROR_READ/WRITE_FAILED
   * code is case of failure, or ERROR_WRONG_DEVICE_MODEL when the target device
   * is not recognized as LPS22HB.s
   */
  int custom_config(uint8_t &CTRL_REG1_value, uint8_t &CTRL_REG2_value,
                    uint8_t &CTRL_REG3_value);

  /**
   * @brief Provides the last pressure reading in hPa.
   *
   * @return float Pressure value in hPa.
   */
  float last_pressure_reading() const { return m_pressure_reading; };

  /**
   * @brief Provides the last temperature reading in degrees C.
   *
   * @return float Temperature value in degrees C.
   */
  float last_temperature_reading() const { return m_temperature_reading; };

  virtual int
  set_mode_of_operation(ST_Sensor::MODE_OF_OPERATION,
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

private:
  /**
   * @brief Puts the device is in power-down mode. Only one shot measurement is
   * possible.
   *
   * @return 0 in case the new CTRL_REG1 value is sent to the sensor
   * successfully, error code ERROR_WRITE_FAILED when there is an issue with
   * sending data to the sensor.
   */
  int enable_one_shot_mode(void);

  /**
   * @brief Starts a one shot measurement by setting bit in CTRL_REG2 register.
   *
   * @return 0 new register value successfully sent, ERROR_WRITE_FAILED in case
   * of an error
   */
  int do_one_shot_measurement(void);

  /**
   * @brief Reads the values of the output pressure and temperature registers.
   * The reading is done in a loop, waiting for bits T_DA and P_DA from STATUS
   * register to be set to 1. If this does not happened in
   * SENSOR_READING_WATCHDOG_COUNTER number of STATUS register checks, the
   * function returns with ERROR_SENSOR_READING_TIME_OUT code.
   *
   * The function can be used in case of ONE_SHOT and continuous measurements.
   * @return 0 in case of success, error code in case of a R/W failure of
   * reading time out.
   */
  int read_data_registers();

  /**
   * @brief Updates the values of the control register(s) for configuring the
   * continuous mode and defines the output data rate of the sensor.
   *
   * @param[in] output_data_rate the new output data rate supported by the
   * sensor
   *
   * @return 0 in case of success, ERROR_WRITE_FAILED in case of an failure
   */
  int config_continuous_mode(ST_Sensor::OUTPUT_DATA_RATE output_data_rate);
};

#endif /* LPS22HB_H_ */
