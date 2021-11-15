/**
 * @file spi_bus.cpp
 *
 * @brief Implements the IO functions from the SPI namespace.
 *
 * @author Goce Boshkovski
 * @date 8 Aug 2019
 *
 * @copyright GNU General Public License v3
 */

#include <iostream>

#include "spi_bus.h"

#include "setila_errors.h"

extern "C" {
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <linux/spi/spidev.h>
}

int SPI::spi_slave_read(SPI_Bus_Master_Device *spi_master, uint8_t registry, void *buffer, int buffer_size)
{
	if (spi_master == nullptr)
	{
		return ERROR_NOT_ATTACHED_TO_A_BUS;
	}

	return 0;
}

int SPI::spi_slave_write_byte(SPI_Bus_Master_Device *spi_master, uint8_t registry, uint8_t value)
{
	if (spi_master == nullptr)
	{
		return ERROR_NOT_ATTACHED_TO_A_BUS;
	}

	return 0;
}

int SPI::spi_slave_write_data(SPI_Bus_Master_Device *spi_master, uint8_t registry, const void *buffer, int buffer_size)
{
	if (spi_master == nullptr)
	{
		return ERROR_NOT_ATTACHED_TO_A_BUS;
	}

	return 0;
}

int SPI::spi_read_write_data(const SPI_Bus_Master_Device *spi_master, void *transmit_buffer, void *receive_buffer, int buffer_size)
{
	if (spi_master == nullptr)
	{
		return ERROR_NOT_ATTACHED_TO_A_BUS;
	}
	
	struct spi_ioc_transfer spi_transfer = {
		.tx_buf = (unsigned long)transmit_buffer,
		.rx_buf = (unsigned long)receive_buffer,
		.len = static_cast<uint32_t>(buffer_size),
		.speed_hz = spi_master->bus_max_speed_hz(),
		.delay_usecs = spi_master->delay(),
		.bits_per_word = spi_master->bits_per_word(),
	};

	/* send the cmd to start the conversion and read the result */
	if (ioctl(spi_master->bus_master_fd(), SPI_IOC_MESSAGE(1), &spi_transfer) < 0)
	{
		return ERROR_READ_WRTIE_FAILED;
	}

	return 0;
}
