/**
 * @file spi_bus_master.cpp
 *
 * @brief Implements SPI_Bus_Master_Device class.
 *
 * @author Goce Boshkovski
 * @date 4 Aug 2019
 *
 * @copyright GNU General Public License v3
 *
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include "spi_bus_master.h"

#include "setila_errors.h"

int SPI_Bus_Master_Device::configure(SPI_BUS_MODE spi_bus_mode, uint32_t spi_bus_max_speed_hz, uint8_t spi_bits_per_word, uint16_t spi_delay)
{
	int configure_status = 0;

	if (bus_master_fd() == -1)
	{
		return ERROR_OPEN_BUS;
	}

	m_spi_bus_mode = spi_bus_mode;
	m_spi_bus_max_speed_hz = spi_bus_max_speed_hz;
	m_spi_bits_per_word = spi_bits_per_word;
	m_spi_delay = spi_delay;


	/* set SPI mode */
	configure_status = ioctl(bus_master_fd(), SPI_IOC_WR_MODE, &spi_bus_mode);
	if (configure_status < 0)
	{
		return ERROR_CONF_BUS_MASTER_FAILED;
	}

	configure_status = ioctl(bus_master_fd(), SPI_IOC_RD_MODE, &spi_bus_mode);
	if (configure_status < 0)
	{
		return ERROR_CONF_BUS_MASTER_FAILED;
	}

	/* set SPI number of bits per word */
	configure_status = ioctl(bus_master_fd(), SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
	if (configure_status < 0)
	{
		return ERROR_CONF_BUS_MASTER_FAILED;
	}
	
	configure_status = ioctl(bus_master_fd(), SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
	if (configure_status < 0)
	{
		return ERROR_CONF_BUS_MASTER_FAILED;
	}


	/* set SPI maximum clock speed */
	configure_status = ioctl(bus_master_fd(), SPI_IOC_WR_MAX_SPEED_HZ, &spi_bus_max_speed_hz);
	if (configure_status < 0)
	{
		return ERROR_CONF_BUS_MASTER_FAILED;
	}

	configure_status = ioctl(bus_master_fd(), SPI_IOC_RD_MAX_SPEED_HZ, &spi_bus_max_speed_hz);
	if (configure_status < 0)
	{
		return ERROR_CONF_BUS_MASTER_FAILED;
	}

	return configure_status;
}
