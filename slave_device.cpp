/**
 * @file slave_device.cpp
 *
 * @brief Implementation of Slave_Device class.
 *
 * @author Goce Boshkovski
 * @date 19 Jul 2019
 *
 * @copyright GNU General Public License v3
 */

extern "C" {
    #include <stdlib.h>
    #include <time.h>
}

#include "slave_device.h"

#define MS_2_NS		1000000L

Slave_Device::~Slave_Device()
{
}

void Slave_Device::time_delay_ms(unsigned int ms_delay)
{
    struct timespec timer;

    timer.tv_sec = 0;
    timer.tv_nsec = ms_delay * MS_2_NS;

    /* Wait till defined interval expires. */
    nanosleep(&timer, NULL);
}

