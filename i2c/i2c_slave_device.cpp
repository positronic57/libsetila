/**
 * @file i2c_slave_device.cpp
 *
 * @brief Implementation of the I2C_Slave_Device class.
 *
 * @author Goce Boshkovski
 * @date 4 Aug 2019
 *
 * @copyright GNU General Public License v3
 *
 */

extern "C" {
    #include <fcntl.h>
    #include <sys/ioctl.h>
    #include <fcntl.h>
    #include <stdlib.h>
    #include <unistd.h>
    #include <time.h>
}


#include "i2c_slave_device.h"
#include "i2c_bus_master.h"
#include "i2c_bus.h"
#include "setila_errors.h"

int I2C_Slave_Device::attach_to_bus(Bus_Master_Device *bus_master_device)
{
	if (bus_master_device == nullptr)
	{
		return ERROR_ATTACH_TO_BUS;
	}

	if (bus_master_device->bus_type() == BUS_TYPE::I2C_BUS)
	{
		m_bus_master = static_cast <I2C_Bus_Master_Device *> (bus_master_device);
		return 0;
	}

	return ERROR_ATTACH_TO_BUS;
}

void I2C_Slave_Device::dettach_from_master_bus()
{
  m_bus_master = nullptr;
}

int I2C_Slave_Device::read(uint8_t address, void *buffer, int buffer_size)
{
	return I2C::i2c_slave_read(m_bus_master->bus_master_fd(), m_I2C_slave_address, address, buffer, buffer_size);
}

int I2C_Slave_Device::write(uint8_t address, void *buffer, int buffer_size)
{
	return I2C::i2c_slave_write_data(m_bus_master->bus_master_fd(), m_I2C_slave_address, address, buffer, buffer_size);
}

int I2C_Slave_Device::write_byte(uint8_t address, uint8_t value)
{
	return I2C::i2c_slave_write_byte(m_bus_master->bus_master_fd(), m_I2C_slave_address, address, value);
}

int I2C_Slave_Device::read_write(void *transmit_buffer, void *receive_buffer, int buffer_size)
{
	return ERROR_IO_MODE_NOT_SUPPORTED; //I2C does not support full-duplex communication.
}
