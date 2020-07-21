/**
 * @file i2c_slave_device.h
 *
 * @brief Contains the definition of the I2C_Slave_Device class/
 *
 * @author Goce Boshkovski
 * @date 4 Aug 2019
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef I2C_SLAVE_DEVICE_H_
#define I2C_SLAVE_DEVICE_H_

#include <stdint.h>

#include "slave_device.h"
#include "i2c_bus_master.h"

/**
 * \defgroup I2C_SLAVE_DEVICES I2C Slave Devices
 * @ingroup I2C_BUS
 */

/**
 * \class I2C_Slave_Device
 * @ingroup I2C_SLAVE_DEVICES
 *
 * @brief Child of Slave_Device class.
 * Implements the IO functions specific for I2C bus and
 * I2C slave devices.
 *
 * @example BMP085.h practical example of using I2C_Slave_Device class for communication with BOSCH BMP085/180 sensor
 * @example LPS25H.h practical example of using I2C_Slave_Device class for communication with LPS25H sensor
 * @example HTS221.h practical example of using I2C_Slave_Device class for communication with HTS221 sensor
 * @example srf02.h practical example of using I2C_Slave_Device class for communication with SFT02 ultra sound range finder
 */
class I2C_Slave_Device: public Slave_Device
{
private:
  uint8_t m_I2C_slave_address = 0x10;					/**< The salve will be reachable via this master device. */
  I2C_Bus_Master_Device *m_bus_master = nullptr;		/**< Address of the slave device on the I2C bus. */

public:
  I2C_Slave_Device(): Slave_Device(Slave_Device_Type::I2C_SLAVE_DEVICE) {};

  /**
   * @brief The class constructor.
   *
   * @param[in] I2C_slave_address address of the slave device on the I2C bus.
   */
  explicit I2C_Slave_Device(uint8_t I2C_slave_address):
    Slave_Device(Slave_Device_Type::I2C_SLAVE_DEVICE),
    m_I2C_slave_address(I2C_slave_address)
  {};

  /**
   * @brief The class destructor.
   */
  ~I2C_Slave_Device() {};

  /**
   * @brief Read data from I2C slave device starting from a given internal registry.
   *
   * @param[in] address start reading the data from this internal address in the slave device
   * @param[out] buffer data read from the slave device will be written here
   * @param[in] buffer_size number of bites to be read from the device
   *
   * @return 0 in case there are no errors on the bus, error code in case of communication failure
   */
  int read(uint8_t address, void *buffer, int buffer_size) override;

  /**
   * @brief Writes the content of the buffer into the target I2C slave starting from a given internal registry.
   *
   * @param[in] address start writing the data from the input buffer starts form this internal address
   * @param[in] buffer pointer to a buffer of bytes which will be sent to the slave device
   * @param[in] buffer_size size of the buffer in bytes
   *
   * @return int 0 in case there are no errors on the bus, error code in case of communication failure
   */
  int write(uint8_t address, void *buffer, int buffer_size) override;

  /**
   * @brief Writes one byte into an internal registry of the slave device.
   *
   * @param[in] address write the byte into this registry/at this internal slave address
   * @param[in] value 8-bit value which will be sent to the device
   *
   * @return int 0 in case there are no errors on the bus, error code in case of communication failure
   */
  int write_byte(uint8_t address, uint8_t value) override;

  /**
   * @brief A dummy function that implements the read_write() virtual function from the parent class.
   * Full-duplex communication is not supported by I2C bus.
   * This function returns immediately.
   *
   * @param transmit_buffer
   * @param receive_buffer
   * @param buffer_size
   *
   * @return always returns 0
   */
  int read_write(void *transmit_buffer, void *receive_buffer, int buffer_size) override;

  /**
   * @brief Defines the I2C bus master device which will be use for communication with the slave device.
   *
   * @param[in] bus_master_device a pointer to the SPI bus master
   *
   * @return 0 for success, ERROR_ATTACH_TO_BUS in case of a failure.
   */
  int attach_to_bus(Bus_Master_Device *bus_master_device) override;

  /**
   * @brief Removes the I2C bus master defined with the attach_to_bus() function call.
   *
   */
  void dettach_from_master_bus() override;
};

#endif /* I2C_SLAVE_DEVICE_H_ */
