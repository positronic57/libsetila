/*
 * Setila library
 * libsetila.c
 *
 * Copyright (c) 2016  Goce Boshkovski
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

/** @file libsetila.c
 *  @brief Implements the functions defined in the header file of the library.
 *
 * @author Goce Boshkovski
 * @date 20-Feb-16
 * @copyright GNU General Public License v2.
 *
 */
#include <stdio.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>

#include "setila.h"

int sensehat_initSenseHAT(TSenseHAT *pTSenseHAT,char *I2Cbus)
{
	/* Open i2c device on Raspberry Pi for R/W. */
	pTSenseHAT->fd=open(I2Cbus,O_RDWR);
	if (pTSenseHAT->fd<0)
		return ERROR_OPEN_I2C_BUS;	//Failed to connect on I2C bus
	
	/* Init of HTS221 sensor */
	if (sensehat_initHTS221Sensor(pTSenseHAT))
		return ERROR_INIT_HTS221_SENSOR;   // Failed to init HTS221 sensor.

	/* Fetch the calibration table of HST221. Required for calculating the humidity and temperature value. */
	if (sensehat_readHumiditySensorCalibrationTable(pTSenseHAT))
		return ERROR_READ_HTS221_CALIBRATION_TABLE;	// Failed to read the HTS221 calibration table.

	return 0;
}

float sensehat_TemperatureReading(TSenseHAT *pTSenseHAT,TSensorType fromSensor)
{

	switch(fromSensor)
	{
		case (HUMIDITY_SENSOR):
			return pTSenseHAT->HTS221TemperatureReading;
			break;
		case (PRESSURE_SENSOR):
			break;
		default:
			return 1;
	}

	return 0;
}

int sensehat_calculateRealtiveHumidity(TSenseHAT *pTSenseHAT)
{
	int16_t H0_T0_out = 0;
	int16_t H1_T0_out = 0;
	int16_t H_T_out = 0;
	float H0_rh, H1_rh;

	H0_rh=pTSenseHAT->HTS221CalibrationTable[0]/2.0;
	H1_rh=pTSenseHAT->HTS221CalibrationTable[1]/2.0;

	H0_T0_out=pTSenseHAT->HTS221CalibrationTable[7];
	H0_T0_out<<=8;
	H0_T0_out|=pTSenseHAT->HTS221CalibrationTable[6];
	
	H1_T0_out=pTSenseHAT->HTS221CalibrationTable[11];
	H1_T0_out<<=8;
	H1_T0_out|=pTSenseHAT->HTS221CalibrationTable[10];

	H_T_out=pTSenseHAT->HTS221HumidityOut[1];
	H_T_out<<=8;
	H_T_out|=pTSenseHAT->HTS221HumidityOut[0];

	/* convert negative 2's complement values to native negative value */
	if (H0_T0_out&0x8000) H0_T0_out = -((~H0_T0_out)+1);
	if (H1_T0_out&0x8000) H1_T0_out = -((~H1_T0_out)+1);
	if (H_T_out&0x8000) H_T_out = -((~H_T_out)+1);

	pTSenseHAT->HTS221RelativeHumidityReading=((float)((H_T_out - H0_T0_out) * (H1_rh - H0_rh)) / (float)(H1_T0_out - H0_T0_out)) + H0_rh;

	return 0;
}

int sensehat_HTS221calculateTemperature(TSenseHAT *pTSenseHAT)
{
	int16_t T0_out, T1_out, T_out;
	uint16_t tmp;
	float T0_degC, T1_degC;

	T0_out=pTSenseHAT->HTS221CalibrationTable[13];
	T0_out<<=8;
	T0_out|=pTSenseHAT->HTS221CalibrationTable[12];

	T1_out=pTSenseHAT->HTS221CalibrationTable[15];
	T1_out<<=8;
	T1_out|=pTSenseHAT->HTS221CalibrationTable[14];

	T_out=pTSenseHAT->HTS221TemperatureOut[1];
	T_out<<=8;
	T_out|=pTSenseHAT->HTS221TemperatureOut[0];

	/* Convert negative 2's complement values to native negative value */
	if (T0_out&0x8000) T0_out = -((~T0_out)+1);
	if (T1_out&0x8000) T1_out = -((~T1_out)+1);
	if (T_out&0x8000) T_out = -((~T_out)+1);

	/* Calculate the T0_degC coefficient */
	tmp=(pTSenseHAT->HTS221CalibrationTable[5]) & 0x03;
	tmp<<=8;
	tmp|=pTSenseHAT->HTS221CalibrationTable[2];
	T0_degC=tmp/8.0;

	/* Calculate the T1_degC coefficient */
	tmp=(pTSenseHAT->HTS221CalibrationTable[5]) & 0x0C;
	tmp<<=6;
	tmp|=pTSenseHAT->HTS221CalibrationTable[3];
	T1_degC=tmp/8.0;

	pTSenseHAT->HTS221TemperatureReading=((float)((T_out - T0_out)*(T1_degC - T0_degC))/(float)(T1_out - T0_out))+T0_degC;

	return 0;
}

float sensehat_HumidityReading(TSenseHAT *pTSenseHAT)
{
	return pTSenseHAT->HTS221RelativeHumidityReading;
}

float sensehat_PressureReading(TSenseHAT *pTSenseHAT)
{

	return 0;
}

int sensehat_startHumidityMeasurement(TSenseHAT *pTSenseHAT)
{
	struct timespec timer = {
		.tv_sec = 0,
		.tv_nsec = 5000000L,
	};

	/* Start a humidity and temperature measurement */
	if (i2c_write(pTSenseHAT->fd,HUMIDITY_SENSOR_ADDR,HTS221_CTRL_REG2,1))
		return ERROR_HTS221_MEASUREMENT_FAILED;

	/* Wait around 5ms for finishing the measurement and start reading the senor output */
	nanosleep(&timer, NULL);

	/* Read humidity registers. MSB bit of HTS221_HUMIDITY_OUT_L address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (i2c_read(pTSenseHAT->fd,HUMIDITY_SENSOR_ADDR,(HTS221_HUMIDITY_OUT_L | 0x80),pTSenseHAT->HTS221HumidityOut,2))
		return ERROR_HTS221_MEASUREMENT_FAILED;

	/* Read temperature registers. MSB bit of HTS221_TEMP_OUT_L address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (i2c_read(pTSenseHAT->fd,HUMIDITY_SENSOR_ADDR,(HTS221_TEMP_OUT_L | 0x80),pTSenseHAT->HTS221TemperatureOut,2))
		return ERROR_HTS221_MEASUREMENT_FAILED;

	/* Calculate the relative humidity value based on the measurement data. */
	sensehat_calculateRealtiveHumidity(pTSenseHAT);
	/* Calculate the temperature value based on the measurement data. */
	sensehat_HTS221calculateTemperature(pTSenseHAT);

	return 0;
}

int sensehat_readHumiditySensorCalibrationTable(TSenseHAT *pTSenseHAT)
{
	/* Read the content of the calibration table from HTS221 sensor.
	 * The start address of the calibration table is HTS221_CALB_0.
	 * The MSB bit of the register address is set to 1 for
	 * enabling address auto-increment.
	*/
	if(i2c_read(pTSenseHAT->fd,HUMIDITY_SENSOR_ADDR,(HTS221_CALIB_0 | 0x80),pTSenseHAT->HTS221CalibrationTable,16))
		return 1;

	return 0;
}

int sensehat_initHTS221Sensor(TSenseHAT *pTSenseHAT)
{
	uint8_t registryValue;

	/*Check the device ID by reading WHO_AM_I register*/
	if (i2c_read(pTSenseHAT->fd,HUMIDITY_SENSOR_ADDR,HTS221_WHO_AM_I,&registryValue,1))
		return 1;
	if (registryValue!=0xBC)
		return 1;

	/* Set the values of AV_CONF registers to value 0x1B. */
	if (i2c_write(pTSenseHAT->fd,HUMIDITY_SENSOR_ADDR,HTS221_AV_CONF,0x1B))
		return 1;
	/* Set the value of the HTS221_CTRL_REG1 to value 0x84. */
	if (i2c_write(pTSenseHAT->fd,HUMIDITY_SENSOR_ADDR,HTS221_CTRL_REG1,0x84)) 
		return 1;
	/* Set the value of the HTS221_CTRL_REG2 to value 0x00. */
	if (i2c_write(pTSenseHAT->fd,HUMIDITY_SENSOR_ADDR,HTS221_CTRL_REG2,HTS221_CTRL_REG2_DEFAULT_VALUE))
		return 1;
        /* Set the value of the HTS221_CTRL_REG3 to value 0x00. */
        if (i2c_write(pTSenseHAT->fd,HUMIDITY_SENSOR_ADDR,HTS221_CTRL_REG3,HTS221_CTRL_REG3_DEFAULT_VALUE))
                return 1;

	return 0;
}

int i2c_write(int fd,uint8_t i2cAddress,uint8_t registry,uint8_t value)
{
	uint8_t tbuffer[2] = { registry, value };

	/*Select the sensor on the I2C bus with i2c address as a slave device.*/
	if (ioctl(fd,I2C_SLAVE,i2cAddress)<0)
		return 1;

	/* Place the register address and value on I2C bus */
	if (write(fd,tbuffer,2)!=2)
		return 1;

	return 0;
}

int i2c_read(int fd,uint8_t i2cAddress,uint8_t registry,uint8_t *buffer,int size)
{
	/*Select the sensor on the I2C bus with i2c address as a slave device.*/
	if (ioctl(fd,I2C_SLAVE,i2cAddress)<0)
		return 1;

	/* Place the registry address on I2C bus */
	if (write(fd,&registry,1)!=1)
		return 1;

	/* Read the data from the I2C bus */
	if (read(fd,buffer,size)!=size)
		return 1;

	return 0;
}


