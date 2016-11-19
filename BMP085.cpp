 /**
  * @file BMP085.cpp
  * @brief Implementation of BMP085 class.
  *
  * @author Goce Boshkovski
  * @date 27-Apr-16
  * @copyright GNU General Public License v2.
  *
  */

#include "BMP085.h"
#include "setila_errors.h"

BMP085::BMP085(unsigned char SensorAddress):I2CSensor(SensorAddress),pressureReading(0.0),temperatureReading(0.0),oss(standard){}

BMP085::~BMP085() {}

int BMP085::readBMP085CalibrationTable()
{
	return I2CSensor_Read(BMP085_CALIBRATION_TABLE,calibrationTable,22);
}

BMP085::BMP085(unsigned char SensorAddress,BMP085_OverSampling oss):I2CSensor(SensorAddress),pressureReading(0.0),temperatureReading(0.0)
{
	this->oss=oss;
}

int BMP085::initSensor()
{
	return readBMP085CalibrationTable();
}

float BMP085::getTemperatureReading()
{
	return temperatureReading;
}

float BMP085::getPressureReading()
{
	return pressureReading;
}

void BMP085::setOversampling(BMP085_OverSampling oss)
{
	this->oss=oss;
}

int BMP085::measureTemperaturePressure()
{
	long rawTemperature,temperature = 0;
    long rawPressure = 0, pressure = 0;

    long x1, x2, x3, b3, b5, b6 = 0;
    unsigned long b4, b7 = 0;

    short ac1 = createSword(calibrationTable[0],calibrationTable[1]);
    short ac2 = createSword(calibrationTable[2],calibrationTable[3]);
    short ac3 = createSword(calibrationTable[4],calibrationTable[5]);
    unsigned short ac4 = createUword(calibrationTable[6],calibrationTable[7]);
    unsigned short ac5 = createUword(calibrationTable[8],calibrationTable[9]);
    unsigned short ac6 = createUword(calibrationTable[10],calibrationTable[11]);
    short b1 = createSword(calibrationTable[12],calibrationTable[13]);
    short b2 = createSword(calibrationTable[14],calibrationTable[15]);
    short mc = createSword(calibrationTable[18],calibrationTable[19]);
    short md = createSword(calibrationTable[20],calibrationTable[21]);

    /* Send the command to start the temperature measurement. */
    if (I2CSensor_Write(BMP085_CONTROL_REG,BMP085_START_TEMPERATURE_MEASUREMENT))
    	return ERROR_BMP085_START_TEMPERATURE_MEASUREMENT_FAILED;

    /* Wait approximately 4.5ms for temperature conversion. */
    tdelay(BMP085_PressureConversionTime[0]);

    /* Read the temperature raw value */
    if (I2CSensor_Read(BMP085_DATA_REG_MSB,rawTemperatureData,2))
    	return ERROR_BMP085_READ_TEMPERATURE_FAILED;
    rawTemperature = (long)((rawTemperatureData[0]<<8)+rawTemperatureData[1]);

	/* Start the pressure measurement by setting the value of the BMP085 control register.*/
    if (I2CSensor_Write(BMP085_CONTROL_REG,(BMP085_START_PRESSURE_MEASUREMENT+(oss<<6))))
    	return ERROR_BMP085_START_PRESSURE_MEASUREMENT_FAILED;

	/* Wait for the pressure conversion to be done. */
	tdelay(BMP085_PressureConversionTime[oss]);

	/* Read the pressure raw value. */
	if (I2CSensor_Read(BMP085_DATA_REG_MSB,rawPressureData,2))
		return ERROR_BMP085_READ_PRESSURE_FAILED;
    if (I2CSensor_Read(BMP085_DATA_REG_XLASB,&rawPressureData[2],1))
    	return ERROR_BMP085_READ_PRESSURE_FAILED;
	rawPressure = ((rawPressureData[0]<<16) + (rawPressureData[1]<<8) + rawPressureData[2])>>(8-oss);

	/* Calculate the actual temperature value. */
    x1 = ((rawTemperature - ac6)*ac5)>>15;
    x2 = (mc << 11)/(x1 + md);
    b5 = x1 + x2;
    temperature = (b5 + 8)>>4;
    temperatureReading = temperature * 0.1;

	/* Calculate the actual pressure value in hPa. */
	b6 = b5 - 4000;
	x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
	x2 = (ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = ((( ac1 * 4 + x3 ) << oss) + 2 ) / 4;
	x1 = (ac3 * b6) >> 13;
	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1+x2)+2) >> 2;
	b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;
	b7 = ((unsigned long)rawPressure-b3)*(50000>>oss);
	if (b7<0x80000000)
		pressure=(b7*2)/b4;
	else
		pressure=(b7/b4)*2;
	x1=(pressure >> 8)* (pressure >> 8);
	x1=(x1*3038)>>16;
	x2=(-7357*pressure)>>16;
	pressure+=((x1+x2+3791)>>4);
	pressureReading = (float)pressure/100.0;

	return 0;
}
