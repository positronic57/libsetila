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

#include <iostream>
#include <memory>
#include <string>
#include <string_view>

#include "setila/srf02.h"

int main(void) {

  std::unique_ptr<Bus_Master_Device> i2c_bus_master{
      new Bus_Master_Device("/dev/i2c-1", BUS_TYPE::I2C_BUS)};

  std::unique_ptr<SRF02> srf02{new SRF02(SRF02::I2C_Address)};

  if (i2c_bus_master->open_bus() < 0) {
    std::cout << "Failed to open master bus\n";
    return -1;
  }

  srf02->attach_to_bus(i2c_bus_master.get());

  SRF02::Status find_status =
      srf02->srf02_find_object(SRF02::Command::REAL_RANGING_MODE_cm);
  if (find_status != SRF02::Status::OBJECT_FOUND) {
    std::cout << "Error: " << srf02->srf02_status_to_string(find_status)
              << '\n';
    return -1;
  } else {
    std::cout << "Object detected at range: " << srf02->distance() << "cm.\n";
  }

  return 0;
}
