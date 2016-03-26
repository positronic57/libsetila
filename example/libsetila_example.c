/*
 * Setila Library Example
 * libsetila_example.c
 *
 * Copyright (c) 2016  Goce Boshkovski
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

/** @file libsetila_example.c
 *  @brief Demonstrates the use of libsetila library for 
 *  temperature and relative humidity measurement.
 *
 * @author Goce Boshkovski
 * @date 20-Feb-16
 * @copyright GNU General Public License v2.
 *
 */
#include <stdio.h>
#include <stdlib.h>

#include "setila.h"

int main(void)
{
	int error_code;
	TSenseHAT *pTSenseHAT;

	pTSenseHAT=(TSenseHAT *)malloc(sizeof(TSenseHAT));

	/* Init Sense HAT sensors. */	
	error_code=sensehat_initSenseHAT(pTSenseHAT,"/dev/i2c-1");
	if (error_code)
	{
		printf("Failed to init Sense HAT. Error code: %d.\n",error_code);
		return error_code;
	}

	printf("HTS221 sensor detected.\n");

	putchar('\n');

	/* Start humidity and temperature measurement with HTS221 humidity sensor. */
	if (sensehat_startHumidityMeasurement(pTSenseHAT))
	{
		printf("Humidity and temperature measurement failed with HTS221.");
		return ERROR_HTS221_MEASUREMENT_FAILED;
	}

	printf("Relative humidity R=%f[%%rH]\n",sensehat_HumidityReading(pTSenseHAT));
	
	printf("Temperature T=%f[%cC]\n",sensehat_TemperatureReading(pTSenseHAT,HUMIDITY_SENSOR),248);

	return 0;
}
