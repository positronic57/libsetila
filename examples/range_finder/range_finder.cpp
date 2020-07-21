/**
 * @file range_finder.cpp
 *
 * @brief Measuring distance with SRF02 ultra sonic range finder using libsetila2.
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 */

#include <string>
#include <iostream>

#include "setila/setila.h"

int main(void)
{
    Bus_Master_Device *i2c_bus_master = new Bus_Master_Device("/dev/i2c-1", BUS_TYPE::I2C_BUS);

    SRF02 *srf02 = new SRF02(0x70);
    int status;

    if (i2c_bus_master->open_bus() < 0)
    {
        std::cout << "Failed to open master bus" << std::endl;
        return -1;
    }

    srf02->attach_to_bus(i2c_bus_master);
    
    status = srf02->srf02_find_object(SRF02_Command::REAL_RANGING_MODE_cm);

    if (status < 0)
    {
    	std::cout << "Error reading SRF02" << std::endl;
    }
    else
    {
    	std::cout << "Object detected at range: "<< srf02->distance() << "cm." << std::endl;
    }

    delete srf02;
    delete i2c_bus_master;

    return 0;
}

