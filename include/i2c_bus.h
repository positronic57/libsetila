/**
 * @file i2c_bus.h
 *
 * @brief Definition of I2C name space
 *
 * @author Goce Boshkovski
 * @date 19 Jul 2019
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef I2C_BUS_H_
#define I2C_BUS_H_

#include <stdint.h>

/**
 * @brief Namespace with functions that implement IO requests for I2C bus.
 * Requests are made by the I2C slave device object using its read/write functions.
 */
namespace I2C {
	/**
	 * @brief Read data from I2C slave device starting from a given internal registry.
	 *
	 * @param[in] i2c_master a file descriptor of the bus master device file
	 * @param[in] i2c_slave_address address of the slave device on the I2C bus
	 * @param[in] registry start reading the data from this internal address in the slave device
	 * @param[out] buffer data read from the slave device will be written here
	 * @param[in] buffer_size number of bites to be read from the device
	 *
	 * @return 0 in case there are no errors on the bus, error code in case of communication failure
	 */
	extern int i2c_slave_read(int i2c_master, uint8_t i2c_slave_address, uint8_t registry, void *buffer, int buffer_size);

	/**
	 * @brief Writes one byte into an internal registry of the slave device.
	 *
	 * @param[in] i2c_master a file descriptor of the bus master device file
	 * @param[in] i2c_slave_address address of the slave device on the I2C bus
	 * @param[in] registry write the byte into this registry/at this internal slave address
	 * @param[in] value 8-bit value which will be sent to the device
	 *
	 * @return int 0 in case there are no errors on the bus, error code in case of communication failure
	 */
	extern int i2c_slave_write_byte(int i2c_master, uint8_t i2c_slave_address, uint8_t registry, uint8_t value);

	/**
	 * @brief Writes the content of the buffer into the target I2C slave starting from a given internal registry.
	 *
	 * @param[in] i2c_master a file descriptor of the bus master device file
	 * @param[in] i2c_slave_address address of the slave device on the I2C bus
	 * @param[in] registry start writing the data from the input buffer starts form this internal address
	 * @param[in] buffer pointer to a buffer of bytes which will be sent to the slave device
	 * @param[in] buffer_size size of the buffer in bytes
	 *
	 * @return int 0 in case there are no errors on the bus, error code in case of communication failure
	 */
	extern int i2c_slave_write_data(int i2c_master, uint8_t i2c_slave_address, uint8_t registry, const void *buffer, int buffer_size);
}

#endif /* I2C_BUS_H_ */
