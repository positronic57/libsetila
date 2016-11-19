 /**
  * @file I2CBus.cpp
  * @brief Implementation of the I2CBus class.
  *
  * @author Goce Boshkovski
  * @date 27-Apr-16
  * @copyright GNU General Public License v2.
  *
  */

#include <cstring>

#include "I2CBus.h"
#include "setila_errors.h"

extern "C"
{
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
}

I2CBus::I2CBus(const char *i2cbus)
{
	fd=-1;
	this->i2cbus = new char[strlen(i2cbus)+1];
	strcpy(this->i2cbus,i2cbus);
}

I2CBus::~I2CBus() {
	delete i2cbus;
	closeI2CBus();
}

I2CBus::I2CBus(const I2CBus &I2CBusSource)
{
	fd=I2CBusSource.fd;

	if (I2CBusSource.i2cbus)
	{
		i2cbus = new char[strlen(I2CBusSource.i2cbus)];
		strcpy(i2cbus,I2CBusSource.i2cbus);
	}
	else
		i2cbus=NULL;
}

I2CBus &I2CBus::operator=(const I2CBus &I2CBusSource)
{
	if (this == &I2CBusSource)
		return *this;

	/* Remove the old content of the i2cbus. */
	delete[] i2cbus;

	if (I2CBusSource.i2cbus)
	{
		this->i2cbus = new char[strlen(i2cbus)+1];
		strcpy(i2cbus,I2CBusSource.i2cbus);
	}
	else
		i2cbus=0;

	return *this;
}

int I2CBus::closeI2CBus()
{
	return close(fd);
}

int I2CBus::openI2CBus()
{
	fd=open(i2cbus,O_RDWR);

	if (fd==-1)
		return ERROR_OPEN_I2C_BUS;
	else
		return 0;
}

int I2CBus::getI2CBus()
{
	return fd;
}

