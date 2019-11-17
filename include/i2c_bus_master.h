/**
 * @file i2c_bus_master.h
 *
 * @brief Contains the definition of I2C_Bus_Master_Device class.
 *
 * @author Goce Boshkovski
 * @date 4 Aug 2019
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef I2C_BUS_MASTER_H_
#define I2C_BUS_MASTER_H_

#include "bus_master_device.h"

/**
 * \class I2C_Bus_Master_Device
 * @ingroup I2C_BUS_MASTER
 *
 * @brief Child class of Bus_Master_Device class which implements
 * function calls specific for I2C bus.
 */
class I2C_Bus_Master_Device: public Bus_Master_Device
{
public:
	I2C_Bus_Master_Device(): Bus_Master_Device("/dev/i2c-1", BUS_TYPE::I2C_BUS) {};
	/**
	 * @brief Class constructor. Sets the value of the bus type in the master class to I2C_BUS.
	 *
	 * @param[in] bus_master_device_file_name bus type managed by the master driver. Required by constructor of the parent class
	 */
	explicit I2C_Bus_Master_Device(const std::string &bus_master_device_file_name):
		Bus_Master_Device(bus_master_device_file_name, BUS_TYPE::I2C_BUS)
	{};

	/**
	 * @brief Default class destructor.
	 */
	~I2C_Bus_Master_Device() {};
};


#endif /* I2C_BUS_MASTER_H_ */
