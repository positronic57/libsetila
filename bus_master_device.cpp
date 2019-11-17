/**
 * @file bus_master_device.cpp
 *
 * @brief Implementation of Bus_Master_Device
 *
 * @date 4 Aug 2019
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 *
 */

#include <string>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include "bus_master_device.h"
#include "setila_errors.h"

Bus_Master_Device::Bus_Master_Device(const std::string &bus_master_device_file_name, BUS_TYPE bus_type):
	m_bus_type(bus_type),
	m_bus_master_device_file_name(bus_master_device_file_name)
	//m_number_of_salve_devices(0)
{
	//m_bus_master_fd = -1;
}

Bus_Master_Device::~Bus_Master_Device()
{
  if (m_bus_master_fd != -1)
  {
    this->close_bus();
  }
}

int Bus_Master_Device::open_bus(void)
{
	m_bus_master_fd = open(m_bus_master_device_file_name.c_str(), O_RDWR);

	return (m_bus_master_fd != -1) ? 0 : ERROR_OPEN_BUS;
}

void Bus_Master_Device::close_bus(void)
{
  if (m_bus_master_fd != -1)
  {
    close(m_bus_master_fd);
  }

  m_bus_master_fd = -1;
}

