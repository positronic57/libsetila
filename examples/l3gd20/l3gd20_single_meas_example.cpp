/**
 * @file l3gd20_single_meas_example.cpp
 *
 * @brief Example for reading raw angluar rate values for X, Y and Z axes from
 * L3GD20 without using FIFO.
 *
 * @author Goce Boshkovski
 *
 *
 * @copyright GNU General Public License v3
 *
 */

#include <chrono>
#include <iostream>
#include <thread>

#include "setila/L3GD20.h"

int main() {
  int status = 0;

  Bus_Master_Device *i2c_bus_master =
      new Bus_Master_Device("/dev/i2c-1", BUS_TYPE::I2C_BUS);

  constexpr uint8_t L3GD20_I2C_Address{0x6B};
  L3GD20 *L3GD20_sensor = new L3GD20(Slave_Device_Type::I2C_SLAVE_DEVICE,
                                     i2c_bus_master, L3GD20_I2C_Address);
  do {
    // Open the I2C device for data R/W.
    if (i2c_bus_master->open_bus() < 0) {
      std::cout << "Failed to open master bus\n";
      status = -1;
      break;
    }

    // Set the parameters for the desired configuration of the sensor
    L3GD20_sensor->config.fifo_type = ST_Sensor::FIFO_TYPE::FIFO_DISABLED;
    L3GD20_sensor->config.full_scale = ST_Sensor::FULL_SCALE::FS_250_DPS;
    L3GD20_sensor->config.mode_of_operation =
        ST_Sensor::MODE_OF_OPERATION::OP_NORMAL_MODE;
    L3GD20_sensor->config.output_data_rate =
        ST_Sensor::OUTPUT_DATA_RATE::ODR_95_Hz;

    // Apply the configuration
    status = L3GD20_sensor->apply_config();
    if (status) {
      std::cout << "Error: configure sensor failed with error " << status
                << '\n';
      break;
    }

    // Get sensor readings for angular lates aproximatelly every second
    while (1) {
      status = L3GD20_sensor->get_sensor_readings();
      if (status) {
        std::cout << "Error: get sensor data failed with error " << status
                  << '\n';
        break;
      }

      std::this_thread::sleep_for(std::chrono::seconds(1));

      std::cout << "raw angular rates: "
                << "X = " << L3GD20_sensor->data.angular_rate_X
                << " Y = " << L3GD20_sensor->data.angular_rate_Y
                << " Z = " << L3GD20_sensor->data.angular_rate_Z << std::endl;
      std::cout
          << "in milli degrees per second [mdps]: "
          << "X = "
          << L3GD20::angular_rate_in_mdps(L3GD20_sensor->config,
                                          L3GD20_sensor->data.angular_rate_X)
          << " Y = "
          << L3GD20::angular_rate_in_mdps(L3GD20_sensor->config,
                                          L3GD20_sensor->data.angular_rate_Y)
          << " Z = "
          << L3GD20::angular_rate_in_mdps(L3GD20_sensor->config,
                                          L3GD20_sensor->data.angular_rate_Z)
          << '\n';
    }
  } while (0);

  return status;
}
