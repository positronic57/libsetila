 /**
  * @file HTS221.cpp
  * @brief Implementation of the HTS221 class.
  *
  * @author Goce Boshkovski
  * @date 01-May-16
  * @copyright GNU General Public License v2.
  *
  */

#include <cinttypes>
#include "HTS221.h"
#include "setila_errors.h"


HTS221::HTS221(unsigned char sensorAddress):I2CSensor(sensorAddress),temperatureReading(0.0),humidityReading(0.0), CTRL_REG1(0x00){}

HTS221::~HTS221() {}

float HTS221::TemperatureReading()
{
	return temperatureReading;
}

float HTS221::HumidityReading()
{
	return humidityReading;
}

int HTS221::calculateTemperature()
{
	int16_t T0_out, T1_out, T_out;
	uint16_t tmp;
	float T0_degC, T1_degC;

	T0_out=this->HTS221CalibrationTable[13];
	T0_out<<=8;
	T0_out|=this->HTS221CalibrationTable[12];

	T1_out=this->HTS221CalibrationTable[15];
	T1_out<<=8;
	T1_out|=this->HTS221CalibrationTable[14];

	T_out=this->HTS221TemperatureOut[1];
	T_out<<=8;
	T_out|=this->HTS221TemperatureOut[0];

	/* Convert negative 2's complement values to native negative value */
	if (T0_out&0x8000) T0_out = -((~T0_out)+1);
	if (T1_out&0x8000) T1_out = -((~T1_out)+1);
	if (T_out&0x8000) T_out = -((~T_out)+1);

	/* Calculate the T0_degC coefficient */
	tmp=(this->HTS221CalibrationTable[5]) & 0x03;
	tmp<<=8;
	tmp|=this->HTS221CalibrationTable[2];
	T0_degC=tmp/8.0;

	/* Calculate the T1_degC coefficient */
	tmp=(this->HTS221CalibrationTable[5]) & 0x0C;
	tmp<<=6;
	tmp|=this->HTS221CalibrationTable[3];
	T1_degC=tmp/8.0;

	this->temperatureReading=((float)((T_out - T0_out)*(T1_degC - T0_degC))/(float)(T1_out - T0_out))+T0_degC;

	return 0;
}

int HTS221::calculateRealtiveHumidity()
{
	int16_t H0_T0_out = 0;
	int16_t H1_T0_out = 0;
	int16_t H_T_out = 0;
	float H0_rh, H1_rh;

	H0_rh=this->HTS221CalibrationTable[0]/2.0;
	H1_rh=this->HTS221CalibrationTable[1]/2.0;

	H0_T0_out=this->HTS221CalibrationTable[7];
	H0_T0_out<<=8;
	H0_T0_out|=this->HTS221CalibrationTable[6];

	H1_T0_out=this->HTS221CalibrationTable[11];
	H1_T0_out<<=8;
	H1_T0_out|=this->HTS221CalibrationTable[10];

	H_T_out=this->HTS221HumidityOut[1];
	H_T_out<<=8;
	H_T_out|=this->HTS221HumidityOut[0];

	/* convert negative 2's complement values to native negative value */
	if (H0_T0_out&0x8000) H0_T0_out = -((~H0_T0_out)+1);
	if (H1_T0_out&0x8000) H1_T0_out = -((~H1_T0_out)+1);
	if (H_T_out&0x8000) H_T_out = -((~H_T_out)+1);

	this->humidityReading=((float)((H_T_out - H0_T0_out) * (H1_rh - H0_rh)) / (float)(H1_T0_out - H0_T0_out)) + H0_rh;

	return 0;
}


int HTS221::doOneShotMeasurement()
{
	/* Start a humidity and temperature measurement */
	if (I2CSensor_Write(HTS221_CTRL_REG2,1))
		return ERROR_HTS221_MEASUREMENT_FAILED;

	/* Wait around 5ms for finishing the measurement and start reading the sensor output */
	tdelay(5000000L);

	/* Read humidity registers. MSB bit of HTS221_HUMIDITY_OUT_L address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (I2CSensor_Read((HTS221_HUMIDITY_OUT_L | 0x80),HTS221HumidityOut,2))
		return ERROR_HTS221_MEASUREMENT_FAILED;

	/* Read temperature registers. MSB bit of HTS221_TEMP_OUT_L address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (I2CSensor_Read((HTS221_TEMP_OUT_L | 0x80),HTS221TemperatureOut,2))
		return ERROR_HTS221_MEASUREMENT_FAILED;

	/* Calculate the relative humidity value based on the measurement data. */
	calculateRealtiveHumidity();
	/* Calculate the temperature value based on the measurement data. */
	calculateTemperature();

	return 0;
}

int HTS221::readSensorCalibrationTable()
{

	if(I2CSensor_Read((HTS221_CALIB_0 | 0x80),HTS221CalibrationTable,16))
		return 1;

	return 0;
}

int HTS221::initSensor(unsigned char AV_CONF_value,unsigned char CTRL_REG1_value,unsigned char CTRL_REG2_value,unsigned char CTRL_REG3_value)
{
	unsigned char registryValue;

	/*Check the device ID by reading WHO_AM_I register*/
	if (I2CSensor_Read(HTS221_WHO_AM_I,&registryValue,1))
		return ERROR_INIT_HTS221_SENSOR;
	if (registryValue!=0xBC)
		return ERROR_INIT_HTS221_SENSOR;

	/* Read HTS221 calibration table. */
	if (readSensorCalibrationTable())
		return ERROR_READ_HTS221_CALIBRATION_TABLE;

	/* Set the values of AV_CONF registers. */
	if (I2CSensor_Write(HTS221_AV_CONF,AV_CONF_value))
		return ERROR_INIT_HTS221_SENSOR;
	/* Set the value of the HTS221_CTRL_REG1. */
	if (I2CSensor_Write(HTS221_CTRL_REG1,CTRL_REG1_value))
		return ERROR_INIT_HTS221_SENSOR;
	/* Set the value of the HTS221_CTRL_REG2. */
	if (I2CSensor_Write(HTS221_CTRL_REG2,CTRL_REG2_value))
		return ERROR_INIT_HTS221_SENSOR;
	/* Set the value of the HTS221_CTRL_REG3. */
	if (I2CSensor_Write(HTS221_CTRL_REG3,CTRL_REG3_value))
    		return ERROR_INIT_HTS221_SENSOR;

	return 0;
}

int HTS221::setOneShotMode(void)
{
	return initSensor(0x1B,0x84,0x00,0x00);
}

int HTS221::getSensorReadings(void)
{
	/* Read humidity registers. MSB bit of HTS221_HUMIDITY_OUT_L address is set to 1 forr
	 * enabling address auto-increment.
	 */
	if (I2CSensor_Read((HTS221_HUMIDITY_OUT_L | 0x80),HTS221HumidityOut,2))
		return ERROR_HTS221_MEASUREMENT_FAILED;

	/* Read temperature registers. MSB bit of HTS221_TEMP_OUT_L address is set to 1 for
	 * enabling address auto-increment.
	 */
	if (I2CSensor_Read((HTS221_TEMP_OUT_L | 0x80),HTS221TemperatureOut,2))
		return ERROR_HTS221_MEASUREMENT_FAILED;

	/* Calculate the relative humidity value based on the measurement data. */
	calculateRealtiveHumidity();
	/* Calculate the temperature value based on the measurement data. */
	calculateTemperature();

	return 0;
}

int HTS221::powerDown(void)
{
	/* Read the value of the CTRL_REG1 */
	if (I2CSensor_Read(HTS221_CTRL_REG1, &CTRL_REG1, 1))
		return ERROR_I2C_READ_FAILED;

	/* Write the new value of the CTRL_REG1 with PD bit set to 0. */
	if (I2CSensor_Write(HTS221_CTRL_REG1, (CTRL_REG1 | (~(1 << HTS221_CTRL_REG1_PD)))))
		return ERROR_I2C_WRITE_FAILED;

	return 0;
}

int HTS221::powerUp(void)
{
	/* Set the value of the PD bit from CTRL_REG1 to 1. */
	if (I2CSensor_Write(HTS221_CTRL_REG1, (CTRL_REG1 | (1 << HTS221_CTRL_REG1_PD))))
		return ERROR_I2C_WRITE_FAILED;

	return 0;
}


