/**
 * @file slave_device.h
 *
 * @brief Contains the definition of Slave_Device class.
 *
 * @author Goce Boshkovski
 * @date 19 Jul 2019
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef SLAVE_DEVICE_H_
#define SLAVE_DEVICE_H_

#include <stdint.h>

/**
 * \enum Slave_Device_Type
 *
 * @brief Defines the type of supported slave types
 * based on the underlying bus.
 */
enum class Slave_Device_Type: int {
  GENERIC_SLAVE_DEVICE = 0,		/**< A generic device type. A dummy value, not used in practice. */
  I2C_SLAVE_DEVICE,        		/**< Slave device connected to I2C bus. */
  SPI_SLAVE_DEVICE         		/**< A slave device connected to SPI bus. */
};

class Bus_Master_Device;

/**
 * \class Slave_Device
 *
 * @brief Base class that represents a slave device.
 * It defines the common properties and IO operations
 * for any slave device independent of the bus type
 * they are connected to.
 */
class Slave_Device
{
private:
  Slave_Device_Type m_slave_device_type = Slave_Device_Type::GENERIC_SLAVE_DEVICE;	/**< Defines the slave device type based on the underlying bus. */

public:
  Slave_Device() = default;

  /**
   * @brief The class destructor.
   */
  virtual ~Slave_Device();

  /**
   * @brief The class constructor.
   *
   * @param[in] slave_device_type defines the slave device type based on the underlying bus
   */
  explicit Slave_Device(Slave_Device_Type slave_device_type):
    m_slave_device_type(slave_device_type)
  {};

  /**
   * @brief Get the slave device type.
   *
   * @return the type of the slave device
   */
  const Slave_Device_Type slace_device_type() const { return m_slave_device_type; };

  /**
   * @brief Assigns a bus muster device to the slave device.
   * A virtual function, implemented in the child class(es) based on the slave device type.
   *
   * @param bus_master_device a pointer to the object that represents the bus master device
   * @return 0 for a non null bus master device pointer and a correct bus master type, -1 otherwise
   */
  virtual int attach_to_bus(Bus_Master_Device *bus_master_device) = 0;

  /**
   * @brief Removes the connection with the bus master object.
   * A virtual function, implemented in the child class(es).
   */
  virtual void dettach_from_master_bus() = 0;

  /**
   * @brief A generic function for reading data from the slave device.
   * Each slave implements it's own read method based on the underlying bus.
   *
   * @param[in] address the data will be read from the slave device starting form this internal address/register
   * @param[out] buffer input buffer (contains the data read from the slave device)
   * @param[in] buffer_size size of the input buffer (maximal number of bites that will be read from the device)
   *
   * @return 0 in case all required IO operations on the bus are successful, -1 otherwise
   */
  virtual int read(uint8_t address, void *buffer, int buffer_size) = 0;

  /**
   * @brief A generic function for sending data to the slave device.
   * Each slave implements it's own write method based on the underlying bus.
   *
   * @param[in] address data will be written into the slave device starting from this internal address/register
   * @param[in] buffer output buffer (contains the data sent to the slave device for writing)
   * @param[in] buffer_size the size of the output buffer
   *
   * @return 0 in case all required IO operations on the bus are successful, -1 otherwise
   */
  virtual int write(uint8_t address, void *buffer, int buffer_size) = 0;

  /**
   * @brief A generic function for sending only one byte to the slave device.
   * Each slave implements it's own write method based on the underlying bus.
   * Useful for changing the value of a specific internal register of the slave device.
   *
   * @param[in] address address of the internal register of the slave device
   * @param[in] value the new value of the target register
   *
   * @return 0 in case all required IO operations on the bus are successful, -1 otherwise
   */
  virtual int write_byte(uint8_t address, uint8_t value) = 0;

  /**
   * @brief A generic function for data exchange between the slave and the master device
   * in case of a full-duplex synchronous bus like SPI.
   *
   * @param[in] transmit_buffer content of this buffer will be sent to the slave device
   * @param[out] receive_buffer the response of the slave device will be written here
   * @param[in] buffer_size the size of the transmit and receive buffers
   *
   * @return 0 in case all required IO operations on the bus are successful, -1 otherwise
   */
  virtual int read_write(void *transmit_buffer, void *receive_buffer, int buffer_size) = 0;

  /**
   * @brief Auxiliary function that introduces a delay in ms.
   * Useful in cases when the salve device requires delay between two successive IO operations
   * on the bus.
   *
   * @param[in] ms_delay time delay size in number of ms.
   */
  void time_delay_ms(unsigned int ms_delay);
};


#endif /* SLAVE_DEVICE_H_ */
