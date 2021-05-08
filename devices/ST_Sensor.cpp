/*
 * ST_Sensor.cpp
 *
 *  Created on: May 1, 2021
 *      Author: bosgoce
 */

#include "ST_sensor.h"
#include "i2c_slave_device.h"
#include "spi_slave_device.h"

ST_Sensor::ST_Sensor()
{

}

ST_Sensor::ST_Sensor(Slave_Device_Type interface_type, Bus_Master_Device *bus_master_device, const uint8_t i2c_address)
{
	m_interface_type = interface_type;

	switch(interface_type) {
	case Slave_Device_Type::I2C_SLAVE_DEVICE:
		m_interface = new I2C_Slave_Device(i2c_address);
		break;
	case Slave_Device_Type::SPI_SLAVE_DEVICE:
		m_interface = new SPI_Slave_Device();
		break;
	case Slave_Device_Type::GENERIC_SLAVE_DEVICE:
	default:
		break;
	}

	m_interface->attach_to_bus(bus_master_device);
}

ST_Sensor::~ST_Sensor()
{
	if (m_interface) {
		delete m_interface;
	}
}
