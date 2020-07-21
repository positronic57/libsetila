/**
 * @file spi_slave_device.h
 *
 * @brief Contains the definition of the SPI_Slave_Device class.
 *
 * @author Goce Boshkovski
 * @date 4 Aug 2019
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef SPI_SLAVE_DEVICE_H_
#define SPI_SLAVE_DEVICE_H_

#include "slave_device.h"
#include "spi_bus_master.h"

/**
 * \defgroup SPI_SLAVE_DEVICES SPI Slave Devices
 * @ingroup SPI_BUS
 */

/**
 * \class SPI_Slave_Device
 * @ingroup SPI_SLAVE_DEVICES
 *
 * @brief Child of Slave_Device class.
 * Implements the IO functions specific for SPI bus.
 * 
 * @example mcp3204.h Use of SPU_Slave_Device class for implementing communication with Microchip MCP304 ADC.
 */
class SPI_Slave_Device: public Slave_Device
{
private:
	SPI_Bus_Master_Device *m_bus_master = nullptr;	/**< The salve will be reachable via this master device. */

public:

	/**
	 * @brief The class constructor.
	 */
	SPI_Slave_Device(): Slave_Device(Slave_Device_Type::SPI_SLAVE_DEVICE) {};

  /**
   * @brief The class destructor.
   */
  ~SPI_Slave_Device() {};

  /**
   * @brief A dummy function that implements the virtual read() function from the parent class.
   * This operation is not supported, use read_write() function instead.
   *
   * @param address
   * @param buffer
   * @param buffer_size
   * @return
   */
  int read(uint8_t address, void *buffer, int buffer_size) override;

  /**
   * @brief A dummy function that implements the virtual write() function from the parent class.
   * This operation is not supported, use read_write() function instead.
   *
   * @param address
   * @param buffer
   * @param buffer_size
   * @return always returns 0
   */
  int write(uint8_t address, void *buffer, int buffer_size) override;

  /**
   * @brief A dummy function that implements the virtual write_byte() function from the parent class.
   * This operation is not supported, use read_write() function instead.
   *
   * @param address
   * @param value
   * @return always returns 0
   */
  int write_byte(uint8_t address, uint8_t value) override;

  /**
   * @brief Implements a synchronous full-duplex communication with the SPI slave.
   * The function sends content of the transmit buffer to the slave, while the slave response
   * is written in the receive buffer. Both buffers have the save size.
   *
   * @param[in] transmit_buffer content of this buffer will be sent to the slave device
   * @param[out] receive_buffer this buffer contains the response from the slave device
   * @param[in] buffer_size the size of the receive and transmit buffers.
   *
   * @return 0 in case there was no error while performing IO calls on SPI bus, -1 otherwise
   */
  int read_write(void *transmit_buffer, void *receive_buffer, int buffer_size) override;


  /**
   * @brief Defines the SPI bus master device which will be use for communication with the slave device.
   *
   * @param[in] bus_master_device a pointer to the SPI bus master
   *
   * @return 0 for success, ERROR_ATTACH_TO_BUS in case of a failure.
   */
  int attach_to_bus(Bus_Master_Device *bus_master_device) override;


  /**
   * @brief Removes the SPI bus master defined with the attach_to_bus() function call.
   *
   */
  void dettach_from_master_bus() override;

  /**
   * @brief Returns the SPI bus master used for communication with the slave device.
   *
   * @return the class member m_bus_master
   */
  SPI_Bus_Master_Device *spi_bus_master() { return m_bus_master; }

};


#endif /* SPI_SLAVE_DEVICE_H_ */
