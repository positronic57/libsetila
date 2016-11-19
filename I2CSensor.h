 /**
  * @file I2CSensor.h
  * @brief A header file from libsetila library. It contains the definition of the I2CSensor class.
  *
  * @author Goce Boshkovski
  * @date 25-Apr-16
  * @copyright GNU General Public License v2.
  *
  */

#ifndef I2CSENSOR_H_
#define I2CSENSOR_H_

/**
 * \class I2CSensor
 *
 * @brief A parent class for all I2C sensors defined in the library.
 *
 */
class I2CSensor {
private:
	unsigned char I2CAddress;	/**< Address of the sensor on the I2C bus. */
	int i2cbus;	/**< File descriptor used for accessing the I2C bus on the system where the sensor is connected. */

protected:
	/**
	 * @brief Time delay function.
	 *
	 * The time interval length is defined by the input argument.
	 *
	 * @param[in] interval defines the length of the time delay.
	 */
	virtual void tdelay(long interval);

	/**
	 * @brief Reads 'nbytes' number of bytes from the sensor starting from address 'registry' into the buffer 'buffer'.
	 *
	 * In case the number of bytes read from the sensor is less then requested, the function returns the error
	 * ERROR_I2C_READ_FAILED.
	 *
	 * @param[in] registry internal registry address
	 * @param[out] buffer holds the data read from the sensor
	 * @param[in] nbytes number of bytes requested from the sensor
	 * @return int 0 in case of success, ERROR_I2C_READ_FAILED for a failure
	 */
	virtual int I2CSensor_Read(unsigned char registry,unsigned char *buffer,int nbytes);

	/**
	 * @brief Writes the data 'value' in the register with internal address registry.
	 *
	 * @param[in] registry the internal address of the registry
	 * @param[in] value the new desired content of the sensor registry
	 * @return int 0 in case of success, ERROR_I2C_WRITE_FAILED for a failure
	 */
	virtual int I2CSensor_Write(unsigned char registry,unsigned char value);

public:
	/**
	 * @brief The constructor of the class.
	 *
	 * If not provided, the I2C address of the sensor is set to 0x01.
	 * The class member 'i2cbus' is set to value -1.
	 * @param[in] I2CAddr address of the sensor on the I2C bus
	 */
	I2CSensor(unsigned char I2CAddr=0x01);

	/**
	 * The destructor of the class.
	 */
	virtual ~I2CSensor();

	/**
	 *
	 * @return
	 */
	unsigned char getSensorAddress();

	/**
	 *
	 * @param I2CAddress
	 */
	void setSensorAddress(unsigned char I2CAddress);

	/**
	 * @brief Associate the sensor with the I2C bus which it physically belongs to.
	 *
	 * The file descriptor of the opened I2C bus is provided by I2CBus::getI2CBus().
	 * The bus must be already opened using I2CBus::openI2CBus().
	 *
	 * @param[in] i2cbus represents the file descriptor of the opened I2C bus provided by I2CBus::getI2CBus().
	 * @return int 0 for success, ERROR_ATTACH_TO_I2CBUS in case of a failure
	 */
	int attachSensorToI2CBus(int i2cbus);

};

#endif /* I2CSENSOR_H_ */
