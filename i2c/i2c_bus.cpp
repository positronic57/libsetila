/**
 * @file i2c_bus.cpp
 *
 * @brief Contains implementation of the functions defined in I2C namespace.
 *
 * @author Goce Boshkovski
 * @date 19 Jul 2019
 *
 * @copyright GNU General Public License v3
 */

#include <stdint.h>


extern "C" {
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
}

#include "setila_errors.h"
#include "i2c_bus.h"

int I2C::i2c_slave_read(int i2c_master, uint8_t i2c_slave_address, uint8_t registry, void *buffer, int buffer_size)
{
	if (i2c_master == -1)
	{
		return ERROR_NOT_ATTACHED_TO_A_BUS;
	}

	/*Select the slave device on the I2C bus.*/
	if (ioctl(i2c_master, I2C_SLAVE, i2c_slave_address) < 0)
		return ERROR_I2C_SELECT_SLAVE;

    /* Place the registry address on I2C bus */
	if (write(i2c_master, &registry, 1) != 1)
		return ERROR_WRITE_FAILED;

	/* Read the data from the I2C slave. */
	if (read(i2c_master, buffer, buffer_size) != buffer_size)
		return ERROR_READ_FAILED;

	return 0;
}

int I2C::i2c_slave_write_byte(int i2c_master, uint8_t i2c_slave_address, uint8_t registry, uint8_t value)
{
	uint8_t write_buffer[2] = { registry, value };

	if (i2c_master == -1)
	{
		return ERROR_NOT_ATTACHED_TO_A_BUS;
	}

    /*Select the sensor on the I2C bus with an address as a slave device.*/
	if (ioctl(i2c_master, I2C_SLAVE, i2c_slave_address) < 0)
		return ERROR_I2C_SELECT_SLAVE;

    /* Place the register address and its value on I2C bus */
	if (write(i2c_master, write_buffer, 2) != 2)
		return ERROR_WRITE_FAILED;

	return 0;
}

int I2C::i2c_slave_write_data(int i2c_master, uint8_t i2c_slave_address, uint8_t registry, const void *buffer, int buffer_size)
{
	uint8_t *write_buffer = (uint8_t *) calloc(buffer_size + 1, sizeof(uint8_t));
	uint8_t return_code = 0;

	if (i2c_master == -1)
	{
		return ERROR_NOT_ATTACHED_TO_A_BUS;
	}

	write_buffer[0] = registry;
	memcpy(write_buffer + 1, buffer, buffer_size);

    /*Select the sensor on the I2C bus with an address as a slave device.*/
	if (ioctl(i2c_master, I2C_SLAVE, i2c_slave_address) < 0)
		return ERROR_I2C_SELECT_SLAVE;

    /* Place the register address and its value on I2C bus */
	if (write(i2c_master, write_buffer, buffer_size + 1) != (buffer_size + 1))
	{
		return_code = ERROR_WRITE_FAILED;
	}

	free(write_buffer);

	return return_code;
}

