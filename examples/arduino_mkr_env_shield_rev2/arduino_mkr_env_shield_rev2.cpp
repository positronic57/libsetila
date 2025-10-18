/**
 * @file ardiuno_mkr_env_schield_rev2.cpp
 *
 * @brief Get temperature, pressure and humidity readings from Arduino MKR ENV
 * Shield Rev2 LPS22HB and HTS221 sensors using libsetila library.
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 *
 */

#include <cstdint>
#include <iostream>

#include "setila/HTS221.h"
#include "setila/LPS22HB.h"

int main() {
  Bus_Master_Device *i2c_bus_master =
      new Bus_Master_Device("/dev/i2c-1", BUS_TYPE::I2C_BUS);

  // ST LPS22HB Pressure and temperature sensor with I2C connection and I2C
  // address 0x5C
  LPS22HB *lps22hb_sensor = new LPS22HB(Slave_Device_Type::I2C_SLAVE_DEVICE,
                                        i2c_bus_master, LPS22HB_ADDR_SA0_0);

  // Humidity and temperature sensor
  constexpr uint8_t HTS221_I2C_ADDRESS{0x5F};
  HTS221 *hts221_sensor = new HTS221(Slave_Device_Type::I2C_SLAVE_DEVICE,
                                     i2c_bus_master, HTS221_I2C_ADDRESS);

  int status{0};

  do {
    if (i2c_bus_master->open_bus() < 0) {
      std::cout << "Failed to open master bus\n";
      status = -1;
      break;
    }

    // Configure LPS22HB for ONE SHOT mode of operation
    status = lps22hb_sensor->set_mode_of_operation(
        ST_Sensor::MODE_OF_OPERATION::OP_ONE_SHOT);
    if (status) {
      std::cout << "LPS22HB sensor initialization failed.\n";
      break;
    }

    // Configure HTS221 for ONE SHOT type of measurements
    status = hts221_sensor->set_mode_of_operation(
        ST_Sensor::MODE_OF_OPERATION::OP_ONE_SHOT);
    if (status) {
      std::cout << "HTS221 sensor initialization failed.\n";
      break;
    }

    // Set HTS221 internal temperature average to 32 and humidity to 64
    status = hts221_sensor->set_resolution(0x04, 0x04);
    if (status) {
      std::cout << "HTS221 sensor set resolution failed.\n";
      break;
    }

    // Measure pressure/temperature and get the readings
    if (lps22hb_sensor->get_sensor_readings()) {
      std::cout << "LPS22HB pressure/temperature measurement failed.\n";
      status = -1;
      break;
    }

    // Measure humidity/temperature and get the readings
    if (hts221_sensor->get_sensor_readings()) {
      std::cout << "HTS221 humidity/temperature measurement failed.\n";
      status = -1;
      break;
    }

    std::cout << "\nReadings from Arduino MKR ENV Shield Rev2:\n\n";
    std::cout << "LPS22HB sensor:\n";
    std::cout << "Pressure P=" << lps22hb_sensor->last_pressure_reading()
              << "[hPa]\n";
    std::cout << "Temperature T=" << lps22hb_sensor->last_temperature_reading()
              << "[°C]\n\n";

    std::cout << "HTS221 sensor:\n";
    std::cout << "Relative Humidity R=" << hts221_sensor->humidity_reading()
              << "[%rH]\n";
    std::cout << "Temperature T=" << hts221_sensor->temperature_reading()
              << "[°C]\n\n";
  } while (0);

  delete lps22hb_sensor;
  delete hts221_sensor;
  delete i2c_bus_master;

  return status;
}
