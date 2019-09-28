/**
 * @file spi_slave_device.cpp
 *
 * @brief Contains the implementation of the SPI_Slave_Device class.
 *
 * @author Goce Boshkovski
 * @date 4 Aug 2019
 *
 * @copyright GNU General Public License v3
 *
 */


#include "spi_slave_device.h"

#include "setila_errors.h"
#include "spi_bus.h"


int SPI_Slave_Device::read(uint8_t address, void *buffer, int buffer_size)
{
	return 0;
}

int SPI_Slave_Device::write(uint8_t address, void *buffer, int buffer_size)
{
	return 0;
}

int SPI_Slave_Device::write_byte(uint8_t address, uint8_t value)
{
	return 0;
}

int SPI_Slave_Device::read_write(void *transmit_buffer, void *receive_buffer, int buffer_size)
{
	if (m_bus_master == nullptr) {
		return ERROR_NOT_ATTCHED_TO_BUS;
	} else if (m_bus_master->bus_master_fd() == -1) {
		return ERROR_NOT_ATTCHED_TO_BUS;
	}

	return SPI::spi_read_write_data(m_bus_master, transmit_buffer, receive_buffer, buffer_size);
}

int SPI_Slave_Device::attach_to_bus(Bus_Master_Device *bus_master_device)
{
	if (bus_master_device == nullptr)
	{
		return ERROR_ATTACH_TO_BUS;
	}

	if (bus_master_device->bus_type() == BUS_TYPE::SPI_BUS)
	{
		m_bus_master = static_cast <SPI_Bus_Master_Device *> (bus_master_device);
		return 0;
	}

	return ERROR_ATTACH_TO_BUS;
}

void SPI_Slave_Device::dettach_from_master_bus()
{
	m_bus_master = nullptr;
}
