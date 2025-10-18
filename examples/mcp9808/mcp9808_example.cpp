/**
 * @file mcp9808_example.cpp
 *
 * @brief Reads temperature from MCP9808 sensor using libsetila library.
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 *
 */

#include <iostream>

// #include "setila/setila_i2c.h"

#include "setila/mcp9808.h"

int main() {
  // Example tested on Pi Zero W rev 1.1 were I2C interface is /dev/i2c-1.
  // Put the correct device name for your platform
  Bus_Master_Device *i2c_bus_master =
      new Bus_Master_Device("/dev/i2c-1", BUS_TYPE::I2C_BUS);

  // MCP9808 temperature sensor with the defaule I2C address of 0x18
  MCP9808 *mcp9808_sensor = new MCP9808(MCP9808_I2C_ADDR);

  int status = 0;

  do {
    if (i2c_bus_master->open_bus() < 0) {
      std::cout << "Failed to open master bus\n";
      status = -1;
      break;
    }

    mcp9808_sensor->attach_to_bus(i2c_bus_master);

    // Configure MCP9808 for continuous conversion mode of operation
    status = mcp9808_sensor->set_mod_of_operation(
        MCP9808::MODE_OF_OPERATION::CONTINUOUS_CONVERSION);
    if (status) {
      std::cout << "Setting mod of operation failed with error " << status
                << '\n';
      break;
    }

    status = mcp9808_sensor->set_resolution(MCP9808::RESOLUTION::RES_0_25_DEG);
    if (status) {
      std::cout << "Setting the resolution for the tenperature measurement "
                   "with error "
                << status << '\n';
      break;
    }

    // Get the temperature readings
    float ambient_temperature = -273.15;

    status = mcp9808_sensor->get_sensor_readings(ambient_temperature);
    if (status) {
      std::cout << "Reading the temperature from the sensor failed with error "
                << status << '\n';
      break;
    }

    std::cout << "\nAmbient temperature reading:\n\n";
    std::cout << "Temperature T=" << ambient_temperature << "[°C]\n\n";
  } while (0);

  delete mcp9808_sensor;
  delete i2c_bus_master;

  return status;
}
