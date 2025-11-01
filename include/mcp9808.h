/**
 * @file mcp9808.h
 *
 * @brief A Header file from libsetila library.
 * It defines MCP9808 class for communication with Microchip MCP9808 temperature
 * sensor.
 *
 * @author Goce Boshkovski
 * @date 23 Oct 2021
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef MCP9808_H
#define MCP9808_H

#include "i2c_slave_device.h"

#include <cstdint>

/**
 * \class MCP9808
 *
 * @ingroup I2C_SLAVE_DEVICES
 *
 * @brief Class which represents MCP9808 sensor with I2C interface
 *
 * @example mcp9808_example.cpp
 */
class MCP9808 : public I2C_Slave_Device {
public:
  /** \defgroup MCP9808_DESC MCP9808
   * @ingroup DEV_REG_CMD
   */

  /** \defgroup MCP9808_I2C_ADDRESSES List of MCP9808 I2C addresses
   * @ingroup MCP9808_DESC
   */
  /* @{ */

  /** @brief I2C address when pins A2, A1 and A0 are connected to the logical 0
   * level*/
  static constexpr uint8_t I2C_ADDR{0x18};
  /* @} */

  /** \defgroup MCP9808_REG_DEF MCP9808 Register address map
   * @ingroup MCP9808_DESC
   */
  /* @{ */
  /** @brief The address of Configuration register */
  static constexpr uint8_t CONFIG_REG{0x01};
  /** @brief The address of Ambient Temperature register */
  static constexpr uint8_t TEMPERATURE_REG{0x05};
  /** @brief The address of Manufacturer ID register */
  static constexpr uint8_t MANU_ID_REG{0x06};
  /** @brief The address of Device ID register */
  static constexpr uint8_t DEV_ID_REG{0x07};
  /** @brief The address of Resolution register */
  static constexpr uint8_t RESOLUTION_REG{0x08};
  /* @} */

  /** \defgroup MCP9808_DEFS MCP9808 common definitions
   * @ingroup MCP9808_DESC
   */
  /* @{ */
  /** @brief The MSB of the Device ID register */
  static constexpr uint8_t DEV_ID{0x04};
  /** @brief The content of Manufacturer ID register */
  static constexpr uint16_t MANU_ID{0x0054};
  /** @brief Manufacturer and device ID bytes combined */
  static constexpr uint16_t MANU_DEV_ID{0x0454};

  /** @brief Shutdown bit of Control register */
  static constexpr uint8_t CONGIF_REG_SHDN_BIT{8};
  /* @} */

private:
  float m_temperature =
      -273.15; /**< Ambient temperature reading from the sensor */
  bool m_device_id_verified = false; /**< Flag for device recognition */
  uint16_t m_config_register =
      0x0000; /**< The last value of the Control register which was successfully
                 sent to the sensor */

  MCP9808() = default;

public:
  /**
   * \enum MODE_OF_OPERATION
   *
   * @brief Modes of operation supported by MCP9808
   */
  enum class MODE_OF_OPERATION : int {
    CONTINUOUS_CONVERSION =
        0,   /**< The sensor continuously measures the temperature. */
    SHUTDOWN /**< Low-power mode. No temperature conversion. Communication with
                the server is still possible */
  };

  /**
   * \enum RESOLUTION
   *
   * @brief List of resolution values supported by MCP9808.
   */
  enum class RESOLUTION : int {
    RES_0_5_DEG = 0x00, /**< Represents temperature resolution of 0.5°C */
    RES_0_25_DEG,       /**< Represents temperature resolution of 0.25°C */
    RES_0_125_DEG,      /**< Represents temperature resolution of 0.125°C */
    RES_0_0625_DEG      /**< Represents temperature resolution of 0.0625°C */
  };

public:
  /**
   * @brief Constructor with the I2C address of the sensor as an argument
   *
   * @param[in] SensorAddress one of the possible MCP9808 I2C addresses
   *
   */
  explicit MCP9808(uint8_t SensorAddress) : I2C_Slave_Device(SensorAddress) {};

  ~MCP9808() {};

  /**
   * @brief Sets the mode of operation and the resolution by adjusting the
   * values of the control and resolution registers
   *
   * @param[in] op_mode the new mode of operation given as enumerator
   * @param[in] resolution the new resolution of the sensor as enumerator
   *
   * @return 0 in case of success, error code in case of a failure in the
   * communication with the sensor
   */
  int set_mod_of_operation(MCP9808::MODE_OF_OPERATION op_mode,
                           MCP9808::RESOLUTION resolution)
      __attribute__((deprecated));

  /**
   * @brief Sets the mode of operation and the resolution by adjusting the
   * values of the control register
   *
   * @param[in] op_mode the new mode of operation given as enumerator
   *
   * @return 0 in case of success, error code in case of a failure in the
   * communication with the sensor
   */
  int set_mod_of_operation(MCP9808::MODE_OF_OPERATION op_mode);

  /**
   * @brief Sets the resolution of the sensor by writing a new value into the
   * resolution register
   *
   * @param[in] resolution the new resolution given as enumerator
   *
   * @return 0 in case of success, error code in case of a failure in the
   * communication with the sensor
   */
  int set_resolution(MCP9808::RESOLUTION resolution);

  /**
   * @brief Reads the content of the ambient temperature register and stores it
   * in the m_temperature class member
   *
   * @return 0 in case there is no error in the I2C communication with the
   * sensor, error code for opposite
   */
  int get_sensor_readings() __attribute__((deprecated));

  /**
   * @brief Reads the content of the ambient temperature register and stores it
   * in the m_temperature class member
   *
   * @param[in,out] ambient_temperature holds the value of the temperature
   * reading
   *
   * @return 0 in case there is no error in the I2C communication with the
   * sensor, error code for opposite
   */
  int get_sensor_readings(float &ambient_temperature);

  /**
   * @brief Returns the temperature value stored in the m_temperature class
   * member
   *
   * @return the value of the class member m_temperature
   */
  float ambient_temperature() const __attribute__((deprecated)) {
    return m_temperature;
  };

private:
  /**
   * @brief Reads the value of the device ID register and compares it with the
   * well known device ID of MCP9808.
   *
   * Used by the public function members to check if the sensor is an actual
   * MCP9808.
   *
   * @return 0 in case of success, ERROR_WRONG_DEVICE_MODEL for non matching
   * device ID or read error for communication issue
   */
  int verify_device_id();

  /**
   * @brief Auxiliary function for setting shut down bit in control register to
   * value 1. This activates the shut down mode at MCP9808.
   *
   * @return 0 for successful update of the control register, error code for
   * communication issue
   */
  int shutdown() __attribute__((deprecated));

  /**
   * @brief Auxiliary function for clearing the shut down bit in control
   * register. This activates the continuous temperature measure.
   *
   * @return 0 for successful update of the control register, error code for
   * communication issue
   */
  int continuous_conversion() __attribute__((deprecated));
};

#endif // MCP9808_H
