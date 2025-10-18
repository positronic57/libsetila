/**
 * @file range_finder.cpp
 *
 * @brief Measuring distance with SRF02 ultra sonic range finder using
 * libsetila2.
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 */

#include <cstdint>
#include <iostream>
#include <string>

#include "setila/srf02.h"

int main(void) {
  Bus_Master_Device *i2c_bus_master =
      new Bus_Master_Device("/dev/i2c-1", BUS_TYPE::I2C_BUS);

  constexpr uint8_t SRF02_I2C_Address{0x70};

  SRF02 *srf02 = new SRF02(SRF02_I2C_Address);

  int status{0};

  do {
    if (i2c_bus_master->open_bus() < 0) {
      std::cout << "Failed to open master bus\n";
      status = -1;
      break;
    }

    srf02->attach_to_bus(i2c_bus_master);

    status = srf02->srf02_find_object(SRF02_Command::REAL_RANGING_MODE_cm);
    if (status < 0) {
      std::cout << "Error reading SRF02\n";
    } else {
      std::cout << "Object detected at range: " << srf02->distance() << "cm.\n";
    }

  } while (0);

  delete srf02;
  delete i2c_bus_master;

  return 0;
}
