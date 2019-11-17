/**
 * @file bus_master_device.h
 *
 * @brief Contains the definition of the Bus_Master_Device class.
 *
 * @author Goce Boshkovski
 * @date 4 Aug 2019
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef BUS_MASTER_DEVICE_H_
#define BUS_MASTER_DEVICE_H_

#include <string>

/**
 * \enum BUS_TYPE
 *
 * @brief Enum class for defining the supported bus types
 */
enum class BUS_TYPE: int {
	DUMMY = -1,	/**< Dummy identifier */
	I2C_BUS,   	/**< Identifier for I2C bus */
	SPI_BUS    	/**< Identifier for SPI bus */
};

/**
 * \class Bus_Master_Device
 * @brief Base class for accessing the device file of the bus master driver.
 * Provides the file descriptor for the opened device file to the
 * slave devices.
 *
 */
class Bus_Master_Device
{
private:
	BUS_TYPE m_bus_type = BUS_TYPE::DUMMY;						/**< Defines the bus type: SPI or I2C. */
	int m_bus_master_fd = -1;									/**< File descriptor for data exchange with the bus master driver. */
	std::string m_bus_master_device_file_name = "/dev/null";	/**< Name of the Linux device file created by the bus master driver. */
	int m_number_of_salve_devices = 0;							/**< Number of connected slave devices. */

public:
	Bus_Master_Device() = default;

	/**
	 * @brief Constructor.
	 * @param[in] bus_master_device_file_name name of the Linux device file created by the master driver
	 * @param[in] bus_type	bus type managed by the master driver
	 */
	explicit Bus_Master_Device(const std::string &bus_master_device_file_name, BUS_TYPE bus_type);

	/**
	 * @brief Default destructor.
	 */
	~Bus_Master_Device();

	/**
	 * @brief Opens the device file of the bus driver.
	 * @return int 0 in case of a successful file device open, ERROR_OPEN_BUS otherwise.
	 */
	int open_bus(void);

	/**
	 * @brief Closes the device file of the bus driver.
	 */
	void close_bus(void);

	/**
	 * @brief Provides the values of the file descriptor for accessing the bus driver
	 * to the requesters(slave devices).
	 * @return value of the m_bus_master_fd class member
	 */
	int bus_master_fd() const { return m_bus_master_fd; };

	/**
	 * @brief Gives the number of the slave devices that requested the file descriptor for accessing
	 * the bus master driver.
	 *
	 * @return the value of the m_number_of_slave_devices.
	 */
	int number_of_slave_devices() const { return m_number_of_salve_devices; };

	/**
	 * @brief Returns the information of the bus type.
	 *
	 * @return value of the m_bus_type member.
	 */
	BUS_TYPE bus_type(void) const { return m_bus_type; };

};

#endif
