/**
 * @file LPS25H.h
 *
 * @brief A header file from libsetila library. It defines LPS25H class for
 * communication with LPS25H sensor.
 *
 * @author Goce Boshkovski
 * @date 30 Apr 2016
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef LPS25H_H_
#define LPS25H_H_

#include "ST_sensor.h"

/** \class LPS25H
 *  @ingroup I2C_SLAVE_DEVICES
 *
 *  @brief A class derived from I2CSensor class
 *  that describes LPS25H sensor.
 *
 *  @example pi_sense_hat.cpp
 *
 */
class LPS25H : public ST_Sensor {
public:
  /** @brief Value of the WHO_AM_I register. */
  static constexpr uint8_t ID{0xBD};

  /** \defgroup LPS25H_DESC LPS25H
   * @ingroup DEV_REG_CMD
   */

  /** \defgroup LPS25H_REG_DEF LPS25H Register address map
   * @ingroup LPS25H_DESC
   */
  /* @{ */
  static constexpr uint8_t REG_REF_P_XL{0x08};
  static constexpr uint8_t REG_REF_P_L{0x09};
  static constexpr uint8_t REG_REF_P_H{0x0A};
  static constexpr uint8_t REG_WHO_AM_I{0x0F};
  static constexpr uint8_t REG_RES_CONF{0x10};
  static constexpr uint8_t REG_CTRL_REG1{0x20};
  static constexpr uint8_t REG_CTRL_REG2{0x21};
  static constexpr uint8_t REG_CTRL_REG3{0x22};
  static constexpr uint8_t REG_CTRL_REG4{0x23};
  static constexpr uint8_t REG_INT_CFG{0x24};
  static constexpr uint8_t REG_INT_SOURCE{0x25};
  static constexpr uint8_t REG_STATUS_REG{0x27};
  static constexpr uint8_t REG_PRESS_POUT_XL{0x28};
  static constexpr uint8_t REG_PRESS_OUT_L{0x29};
  static constexpr uint8_t REG_PRESS_OUT_H{0x2A};
  static constexpr uint8_t REG_TEMP_OUT_L{0x2B};
  static constexpr uint8_t REG_TEMP_OUT_H{0x2C};
  static constexpr uint8_t REG_FIFO_CTRL{0x2E};
  static constexpr uint8_t REG_FIFO_STATUS{0x2F};
  static constexpr uint8_t REG_THS_P_L{0x30};
  static constexpr uint8_t REG_THS_P_H{0x31};
  static constexpr uint8_t REG_RPDS_L{0x39};
  static constexpr uint8_t REG_RPDS_H{0x3A};
  /* @} */

  /** \defgroup LPS25H_CTRL_REG1_REG_DESC LPS25H CTRL_REG1 register description
   * @ingroup LPS25H_DESC
   */
  /* @{ */
  static constexpr uint8_t CTRL_REG1_PD_BIT{0x07};
  static constexpr uint8_t CTRL_REG1_ODR2_BIT{0x06};
  static constexpr uint8_t CTRL_REG1_ODR1_BIT{0x05};
  static constexpr uint8_t CTRL_REG1_ODR0_BIT{0x04};
  static constexpr uint8_t CTRL_REG1_DEFF_EN_BIT{0x03};
  static constexpr uint8_t CTRL_REG1_BDU_BIT{0x02};
  static constexpr uint8_t CTRL_REG1_RESET_AZ_BIT{0x01};
  static constexpr uint8_t CTRL_REG1_SIM_BIT{0x00};
  /* @} */

  /** \defgroup LPS25H_CTRL_REG2_REG_DESC LPS25H TRL_REG2 register description
   * @ingroup LPS25H_DESC
   */
  /* @{ */
  static constexpr uint8_t CTRL_REG2_BOOT_BIT{0x07};
  static constexpr uint8_t CTRL_REG2_FIFO_EN_BIT{0x06};
  static constexpr uint8_t CTRL_REG2_WTM_EN_BIT{0x05};
  static constexpr uint8_t CTRL_REG2_FIFO_MEAN_DEC_BIT{0x04};
  static constexpr uint8_t CTRL_REG2_SWRESET_BIT{0x02};
  static constexpr uint8_t CTRL_REG2_ONE_SHOT_BIT{0x00};
  /* @} */

  /** \defgroup LPS25H_STATUS_REG_REG_DESC LPS25H STATUS_REG register
   * description
   * @ingroup LPS25H_DESC
   */
  /* @{ */
  static constexpr uint8_t STATUS_REG_P_OR_BIT{0x05};
  static constexpr uint8_t STATUS_REG_T_OR_BIT{0x04};
  static constexpr uint8_t STATUS_REG_P_DA_BIT{0x01};
  static constexpr uint8_t STATUS_REG_T_DA_BIT{0x00};
  /* @} */

  /** \defgroup LPS25H_RES_CONF_REG_DESC LPS25H RES_CONF register description
   * @ingroup LPS25H_DESC
   */
  /* @{ */
  static constexpr uint8_t RES_CONF_AVGT1_BIT{0x03};
  static constexpr uint8_t RES_CONF_AVGT0_BIT{0x02};
  static constexpr uint8_t RES_CONF_AVGP1_BIT{0x01};
  static constexpr uint8_t RES_CONF_AVGP0_BIT{0x00};
  /* @} */

  /** \defgroup LPS25H_FIFO_CTRL_REG_DESC LPS25H FIFO_CTRL register description
   * @ingroup LPS25H_DESC
   */
  /* @{ */
  static constexpr uint8_t FIFO_CTRL_F_MODE2_BIT{0x07};
  static constexpr uint8_t FIFO_CTRL_F_MODE1_BIT{0x06};
  static constexpr uint8_t FIFO_CTRL_F_MODE0_BIT{0x05};
  /* @} */

  /** \defgroup LPS25H_FIFO_WATERMARKS LPS25H FIFO_MEAN_MODE Sample Size
   * @ingroup LPS25H_DESC
   */
  /* @{ */
  /** @brief WTM_POINT4..0 = 00001 in FIFO_CTRL register. */
  static constexpr uint8_t FIFO_MEAN_MODE_2_SAMPLES{0x01};
  /** @brief WTM_POINT4..0 = 00011 in FIFO_CTRL register. */
  static constexpr uint8_t FIFO_MEAN_MODE_4_SAMPLES{0x03};
  /** @brief WTM_POINT4..0 = 00111 in FIFO_CTRL register. */
  static constexpr uint8_t FIFO_MEAN_MODE_8_SAMPLES{0x07};
  /** @brief WTM_POINT4..0 = 01111 in FIFO_CTRL register. */
  static constexpr uint8_t FIFO_MEAN_MODE_16_SAMPLES{0x0F};
  /** @brief WTM_POINT4..0 = 11111 in FIFO_CTRL register. */
  static constexpr uint8_t FIFO_MEAN_MODE_32_SAMPLES{0x1F};

  /** @brief Enumerated values that represents FIFO_MEAN mode moving average
   * size in measurement samples. */
  enum class NBR_AVERAGED_SAMPLES {
    AVER_SAMPLES_2,  /**< The pressure output in FIFO_MEAN mode is an average of
                        2  samples. */
    AVER_SAMPLES_4,  /**< The pressure output in FIFO_MEAN mode is an average of
                        4  samples. */
    AVER_SAMPLES_8,  /**< The pressure output in FIFO_MEAN mode is an average of
                        8  samples. */
    AVER_SAMPLES_16, /**< The pressure output in FIFO_MEAN mode is an average of
                        16 samples. */
    AVER_SAMPLES_32  /**< The pressure output in FIFO_MEAN mode is an average of
                        32  samples. */
  };
  /* @} */

private:
  bool m_device_id_verified = false;
  float m_pressure_reading =
      0.0; /**<  The last pressure value measured by the sensor. */
  float m_temperature_reading =
      0.0; /**<  The last temperature value measured by the sensor. */
  uint8_t m_CTRL_REG1 = 0x00; /**< Holds the value of the CTRL_REG1 between
                                 configuration changes. */
  uint8_t m_CTRL_REG2 = 0x00; /**< Holds the value of the CTRL_REG2 between
                                 configuration changes. */
  uint8_t m_FIFO_CTRL = 0x00; /**< Holds the value of the FIFO_CTRL between
                                 configuration changes. */

  LPS25H() = delete;

public:
  explicit LPS25H(Slave_Device_Type interface_type,
                  Bus_Master_Device *bus_master_device,
                  uint8_t I2C_slave_address)
      : ST_Sensor(interface_type, bus_master_device, I2C_slave_address) {};

  /**
   * @brief A destructor of the class.
   */
  ~LPS25H() {};

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
   * @brief Same as set_mode_of_operation() from ST_Sensor class, with
   * additional argument that defines the number of averaged samples in case of
   * FIFO_MEAN mode is selected. Calling set_mode_of_operation() without the
   * last argument for FIFO_MEAN mode will set the number of averaged samples
   * to 2.
   *
   * @param[in] mode_of_operation defines the operation mode of the sensor
   * @param[in] output_data_rate defines the output data rate of the sensor
   * @param[in] fifo_mean_samples if FIFO_MEAN mode is given as an value for the
   * argument one, will set the number of averaged samples
   *
   * @return int 0 in case of success,
   * ERROR_WRONG_DEVICE_MODEL/ERROR_WRITE_FAILED in case of a failure
   */
  int set_mode_of_operation(ST_Sensor::MODE_OF_OPERATION mode_of_operation,
                            ST_Sensor::OUTPUT_DATA_RATE output_data_rate,
                            LPS25H::NBR_AVERAGED_SAMPLES fifo_mean_samples);

  /**
   * @brief Provides the last pressure reading in hPa.
   *
   * @return float Pressure value in hPa.
   */
  float pressure_reading() const { return m_pressure_reading; };

  /**
   * @brief Provides the last temperature reading in degrees C.
   *
   * @return float Temperature value in degrees C.
   */
  float temperature_reading() const { return m_temperature_reading; };

  /**
   * @brief The device is reset to the power on configuration by setting bits:
   * SWRESET and BOOT of CTRL_REG1 to ‘1’.
   *
   * @return int 0 on success, ERROR_READ/WRITE_FAILED in case there is an error
   * in the communication with the sensor.
   */
  int SW_reset(void);

private:
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

  /**
   * @brief Enables the FIFO mean mode with FIFO_MEAN decimation disabled and
   * defines the number of averaged samples
   *
   * @param[in] NUM_AVERAGED_SAMPLES number of averaged samples from the FIFO
   * (moving average)
   *
   * @return int 0 in case of success,
   * ERROR_WRITE_FAILED/ERROR_UNSUPPORTED_DEVICE_OPTION_CONFIG in case of a
   * failure
   */
  int config_fifo_mean_mode(LPS25H::NBR_AVERAGED_SAMPLES NUM_AVERAGED_SAMPLES);

  /**
   * @brief Reads the values of the temperature and pressure output registers.
   * It is called by the get_sensor_readings().
   *
   * @return int 0 in case of successful read calls,
   * ERROR_READ_FAILED/ERROR_SENSOR_READING_TIME_OUT otherwise
   */
  int read_data_registers();

  /**
   * @brief Puts the device is in power-down mode by setting
   * bit PD of CTRL_REG1 to value 0.
   *
   * @return 0 in case the new CTRL_REG1 value is sent to the sensor
   * successfully, error codes ERROR_READ_FAILED/ERROR_WRITE_FAILED when there
   * is an issue with reading/sending data to the sensor.
   */
  int power_down(void);

  /**
   * @brief Prepares the sensor for one shot measurements.
   * FIFO will be disabled, output data rate will be set to 0.
   *
   * @return int 0 in case of successful write of the new configuration,
   * ERROR_WRITE_FAILED in case of a failure
   */
  int enable_one_shot_mode();

  /**
   * @brief Starts one shot measurement of pressure and temperature.
   *
   * @return int 0 for success, ERROR_WRITE_FAILED in case update of the control
   * register failed
   */
  int do_one_shot_measurement();
};

#endif /* LPS25H_H_ */
