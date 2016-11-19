 /**
  * @file I2CSensor.cpp
  * @brief Implementation of the I2CSensor class.
  *
  * @author Goce Boshkovski
  * @date 25-Apr-16
  * @copyright GNU General Public License v2.
  *
  */

#include <cstring>
#include "I2CSensor.h"
#include "setila_errors.h"

extern "C"
{
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
}


I2CSensor::I2CSensor(unsigned char I2CAddr):I2CAddress(I2CAddr),i2cbus(-1){}

I2CSensor::~I2CSensor()
{

}

unsigned char I2CSensor::getSensorAddress()
{
	return I2CAddress;
}

void I2CSensor::setSensorAddress(unsigned char I2CAddress)
{
	this->I2CAddress=I2CAddress;
}

int I2CSensor::attachSensorToI2CBus(int i2cbus)
{
	if (i2cbus<=0)
		return ERROR_ATTACH_TO_I2CBUS;

	this->i2cbus=i2cbus;
	return 0;
}

void I2CSensor::tdelay(long interval)
{
    struct timespec timer;

    timer.tv_sec = 0;
    timer.tv_nsec = interval;

    /* Wait till defined interval expires. */
    nanosleep(&timer, NULL);
}

int I2CSensor::I2CSensor_Read(unsigned char registry,unsigned char *buffer,int nbytes)
{
	/*Select the sensor on the I2C bus with an address as a slave device.*/
	if (ioctl(i2cbus,I2C_SLAVE,I2CAddress)<0)
		return ERROR_I2C_READ_FAILED;

	/* Place the registry address on I2C bus */
	if (write(i2cbus,&registry,1)!=1)
		return ERROR_I2C_READ_FAILED;

	/* Read the data from the I2C bus */
	if (read(i2cbus,buffer,nbytes)!=nbytes)
		return ERROR_I2C_READ_FAILED;

	return 0;
}

int I2CSensor::I2CSensor_Write(unsigned char registry,unsigned char value)
{
	unsigned char tbuffer[2] = { registry, value };

	/*Select the sensor on the I2C bus with an address as a slave device.*/
	if (ioctl(i2cbus,I2C_SLAVE,I2CAddress)<0)
		return ERROR_I2C_WRITE_FAILED;

	/* Place the register address and its value on I2C bus */
	if (write(i2cbus,tbuffer,2)!=2)
		return ERROR_I2C_WRITE_FAILED;

	return 0;
}

