/**
 * @file spi_bus.h
 *
 * @brief Contains the definition of the SPI namespace.
 *
 * @author Goce Boshkovski
 * @date 4 Aug 2019
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef SPI_BUS_H_
#define SPI_BUS_H_

#include <stdint.h>

#include "spi_bus_master.h"

/**
 * @brief Namespace with functions that implement IO requests for SPI bus.
 * Requests are triggered by the SPI slave device object using its read/write functions.
 */
namespace SPI {

	/**
	 * @brief Not supported. Defined due to a compatibility with the I2C IO functions.
	 * Returns immediately without any actions.
	 *
	 * @param spi_master
	 * @param registry
	 * @param buffer
	 * @param buffer_size
	 *
	 * @return 0
	 */
	extern int spi_slave_read(SPI_Bus_Master_Device *spi_master, uint8_t registry, void *buffer, int buffer_size);

	/**
	 * @brief Sends two bites to the slave device: address of an internal registry of the slave and the new value
	 * of this registry. Due to the full-duplex of the SPI bus, the slave will answer to the master with two
	 * bits but they will be ignored by this function.
	 *
	 * @param spi_master spi_master a file descriptor of the bus master device file
	 * @param registry write the byte into this registry (at this internal slave address)
	 * @param value this byte will be sent to the slave device
	 *
	 * @return 0 in case there are no errors on the bus, error code in case of communication failure
	 */
	extern int spi_slave_write_byte(SPI_Bus_Master_Device *spi_master, uint8_t registry, uint8_t value);

	/**
	 * @brief Writes data to the slave device starting from internal location in the slave device defined with
	 * the "registry" argument. Due to the full-duplex of the SPI bus, the slave will answer to the master with
	 * the same number of bytes but they will be ignored by this function.
	 *
	 * @param spi_master spi_master a file descriptor of the bus master device file
	 * @param registry write the data starting from this registry/(at this internal slave address)
	 * @param buffer content of this buffer will be sent to the slave device
	 * @param buffer_size size of the transmit buffer
	 *
	 * @return 0 in case there are no errors on the bus, error code in case of communication failure
	 */
	extern int spi_slave_write_data(SPI_Bus_Master_Device *spi_master, uint8_t registry, const void *buffer, int buffer_size);

	/**
	 * @brief SPI is full-duplex synchronous bus. While writing data to the slave (content of the transmit buffer),
	 * in parallel slave answers with data back to the master (content of the receive buffer). The size of the
	 * transmit and receive buffer are the same.
	 *
	 * @param[in] spi_master a file descriptor of the bus master device file
	 * @param[in] transmit_buffer content of this buffer will be sent to the slave device
	 * @param[out] receive_buffer output from the device will end up here
	 * @param[in] buffer_size size of the transmit and receive buffers
	 *
	 * @return 0 in case there are no errors on the bus, error code in case of communication failure
	 */
	extern int spi_read_write_data(const SPI_Bus_Master_Device *spi_master, void *transmit_buffer, void *receive_buffer, int buffer_size);
}

#endif /* SPI_BUS_H_ */
