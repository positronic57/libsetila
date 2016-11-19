 /**
  * @file I2CBus.h
  * @brief A header file from libsetila library. It contains the definition of the I2CBus class.
  *
  * @author Goce Boshkovski
  * @date 01-May-16
  * @copyright GNU General Public License v2.
  *
  */

#ifndef I2CBUS_H_
#define I2CBUS_H_

/** \class I2CBus
 *  @brief A class that represents the I2C bus on the system.
 *
 */
class I2CBus {
private:
	char *i2cbus;	/**<  A Linux device name associated with the existing I2C bus(es). */
	int fd;	/**<  A file descriptor used for accessing the I2C bus. */

public:
    /**
     * @brief A constructor.
     *
     * Initiate the values of the class members:
     * fd is set to a value of -1,while i2cbus is set with the value pass to the
     * constructor as an argument. If not provided, the i2cbus member will be
     * initiated to the default value "/dev/i2c-0".
     *
     * @param[in] i2cbus device name of the I2C bus present on the system.
     */
	I2CBus(const char *i2cbus="/dev/i2c-0");
	/**
	 * @brief A destructor of the class.
	 */
	virtual ~I2CBus();

	/**
	 * The copy constructor of the class due to a pointer as a class member.
	 *
	 * @param[in] I2CBusSource a reference to I2CBus class
	 */
	I2CBus(const I2CBus &I2CBusSource);

	/**
	 * Overloading the assignment operator due to a pointer as a class member.
	 *
	 * @param[in] I2CBusSource a reference to I2CBus class
	 *
	 * @return I2CBus a reference to I2CBus class
	 */
	I2CBus &operator=(const I2CBus &I2CBusSource);

	/**
	 * @brief A member function that returns the value of fd (File Descriptor) member.
	 *
	 * @return int returns the value of the fd class member .
	 */
	int getI2CBus();
	/**
	 * @brief A member function.
	 *
	 * Opens the device specified with i2cbus class member for RD/WR operations.
	 * @return int returns ERROR_OPEN_I2C_BUS for failed attempt to open i2cbus device, a file descriptor for successful attempt.
	 */
	int openI2CBus();
	/**
	 * @brief A member function.
	 *
	 * Closes the file descriptor, so that it no longer refers to any
     * i2c device.
     *
     * @return int returns 0 on success, -1 on failure.
	 */
	int closeI2CBus();

};

#endif /* I2CBUS_H_ */
