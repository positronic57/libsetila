/**
 * @file srf02.cpp
 *
 * @brief Implementation of the SRF02 class.
 *
 * @author Goce Boshkovski
 * @date 24 Aug 2019
 *
 * @copyright GNU General Public License v3
 *
 */
#include <chrono> // for time delay
#include <thread> // for time delay

#include "srf02.h"

SRF02::Status SRF02::srf02_find_object(SRF02::Command measurement_unit) {
  uint8_t receive_buffer[4];

  if ((measurement_unit > SRF02::Command::REAL_RANGING_MODE_us) ||
      (measurement_unit < SRF02::Command::REAL_RANGING_MODE_inch)) {
    return SRF02::Status::UNSUPPORTED_COMMAND;
  }
  /* Initiate a new sonar ranging by writing ranging command in the command
   * register. */
  if (this->write_byte(static_cast<uint8_t>(SRF02::Register::COMMAND),
                       static_cast<uint8_t>(measurement_unit)) < 0) {
    return SRF02::Status::READ_WRITE_ERROR;
  }

  /* Wait till SRF02 does the ranging. According to the data sheet it takes
   * around 65ms. In this time window SRF02 will ignore I2C read/write commands.
   */
  std::this_thread::sleep_for(std::chrono::milliseconds(70));

  /* Read the ranging result together with the software version.
   * Extra data like software version are read only because in the tests done
   * on Raspberry Pi Zero, reading only the ranging result registers produce
   * reading error on the I2C bus.
   */
  if (this->read(static_cast<uint8_t>(SRF02::Register::COMMAND), receive_buffer,
                 4)) {
    return SRF02::Status::READ_WRITE_ERROR;
  }

  m_software_version =
      receive_buffer[static_cast<int>(SRF02::Register::COMMAND)];

  /* Build the range value from the two range registers. */
  m_distance =
      receive_buffer[static_cast<int>(SRF02::Register::RANGE_HIGH_BYTE)];
  m_distance <<= 8;
  m_distance +=
      receive_buffer[static_cast<int>(SRF02::Register::RANGE_LOW_BYTE)];

  if (m_distance == 0x00) {
    return SRF02::Status::NO_OBJECT_IN_RANGE;
  }

  return SRF02::Status::OBJECT_FOUND;
}

SRF02::Status SRF02::srf02_find_object(SRF02::Command measurement_unit,
                                       uint16_t *measured_distance) {
  uint8_t receive_buffer[4];

  if ((measurement_unit > SRF02::Command::REAL_RANGING_MODE_us) ||
      (measurement_unit < SRF02::Command::REAL_RANGING_MODE_inch)) {
    return SRF02::Status::UNSUPPORTED_COMMAND;
  }
  /* Initiate a new sonar ranging by writing ranging command in the command
   * register. */
  if (this->write_byte(static_cast<uint8_t>(SRF02::Register::COMMAND),
                       static_cast<uint8_t>(measurement_unit))) {
    return SRF02::Status::READ_WRITE_ERROR;
  }

  /* Wait till SRF02 does the ranging. According to the data sheet it takes
   * around 65ms. In this time window SRF02 will ignore I2C read/write commands.
   */
  std::this_thread::sleep_for(std::chrono::milliseconds(70));

  /* Read the ranging result together with the software version.
   * Extra data like software version are read only because in the tests done
   * on Raspberry Pi Zero, reading the ranging result registers only produce
   * reading error on the I2C bus. May be this issue is caused by the bus
   * signals level shifter hardware present between the Pi Zero and the sensor.
   */
  if (this->read(static_cast<uint8_t>(SRF02::Register::COMMAND), receive_buffer,
                 4)) {
    return SRF02::Status::READ_WRITE_ERROR;
  }

  m_software_version =
      receive_buffer[static_cast<int>(SRF02::Register::COMMAND)];

  /* Build the range value from the two range registers. */
  m_distance =
      receive_buffer[static_cast<int>(SRF02::Register::RANGE_HIGH_BYTE)];
  m_distance <<= 8;
  m_distance +=
      receive_buffer[static_cast<int>(SRF02::Register::RANGE_LOW_BYTE)];

  *measured_distance = m_distance;

  if (m_distance == 0x00) {
    return SRF02::Status::NO_OBJECT_IN_RANGE;
  }

  return SRF02::Status::OBJECT_FOUND;
}

SRF02::Status SRF02::srf02_set_new_i2c_address(uint8_t new_i2c_address) {
  return SRF02::Status::OPERATION_SUCCESSFUL;
}

std::string_view SRF02::srf02_status_to_string(SRF02::Status srf02_status) {
  switch (srf02_status) {
  case SRF02::Status::READ_WRITE_ERROR:
    return "Communication error. R/W failed.";
    break;
  case SRF02::Status::OPERATION_SUCCESSFUL:
    return "Operation successful.";
    break;
  case SRF02::Status::OBJECT_FOUND:
    return "Object detected. Read range registers for the range value.";
    break;
  case SRF02::Status::NO_OBJECT_IN_RANGE:
    return "No object detected.";
    break;
  case SRF02::Status::READING_RANGE_FAILED:
    return "Reading range failed.";
    break;
  case SRF02::Status::UNSUPPORTED_COMMAND:
    return "Unsupported command.";
    break;
  default:
    break;
  }

  return "UNKOWN Status";
}
