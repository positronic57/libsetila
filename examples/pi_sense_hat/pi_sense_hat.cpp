/**
 * @file pi_sense_hat.cpp
 *
 * @brief Get temperature, pressure and humidity readings from Raspberry Pi
 * Sense HAT LPS25H and HTS221 sensors using libsetila library.
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 *
 */

#include <iostream>
#include <memory>

#include "setila/HTS221.h"
#include "setila/LPS25H.h"

int main() {

  std::unique_ptr<Bus_Master_Device> i2c_bus_master{
      new Bus_Master_Device("/dev/i2c-1", BUS_TYPE::I2C_BUS)};

  constexpr uint8_t lps25h_I2C_address{0x5C};
  constexpr uint8_t hts221_I2C_address{0x5F};

  std::unique_ptr<LPS25H> lps25h_sensor{
      new LPS25H(Slave_Device_Type::I2C_SLAVE_DEVICE, i2c_bus_master.get(),
                 lps25h_I2C_address)};
  std::unique_ptr<HTS221> hts221_sensor{
      new HTS221(Slave_Device_Type::I2C_SLAVE_DEVICE, i2c_bus_master.get(),
                 hts221_I2C_address)};

  int status{0};

  if (i2c_bus_master->open_bus() < 0) {
    std::cout << "Failed to open master bus\n";
    return -1;
  }

  // Set LPS25H internal temperature average to 16 and pressure to 32
  status = lps25h_sensor->set_resolution(0x01, 0x01);
  if (status) {
    std::cout << "HTS221 sensor set resolution failed.\n";
    return status;
  }

  status = lps25h_sensor->set_mode_of_operation(
      ST_Sensor::MODE_OF_OPERATION::OP_FIFO_MEAN_MODE,
      ST_Sensor::OUTPUT_DATA_RATE::ODR_1_Hz,
      LPS25H::NBR_AVERAGED_SAMPLES::AVER_SAMPLES_4);
  if (status) {
    std::cout << "LPS25H sensor initialization failed.\n";
    return status;
  }

  // Set HTS221 internal temperature average to 32 and humidity to 64
  status = hts221_sensor->set_resolution(0x04, 0x04);
  if (status) {
    std::cout << "HTS221 sensor set resolution failed.\n";
    return status;
  }

  // Configure HTS221 for ONE SHOT type of measurements
  status = hts221_sensor->set_mode_of_operation(
      ST_Sensor::MODE_OF_OPERATION::OP_ONE_SHOT);
  if (status) {
    std::cout << "HTS221 sensor initialization failed.\n";
    return status;
  }

  // Get the humidity and temperature readings from the sensor and calculate
  // the current temperature and humidity values.
  if (hts221_sensor->get_sensor_readings()) {
    std::cout << "HTS221 humidity/temperature measurement failed.\n";
    return status - 1;
  }

  // Measure the temperature and the atmospheric pressure with LPS25H
  if (lps25h_sensor->get_sensor_readings()) {
    std::cout << "LPS25H pressure/temperature measurement failed.\n";
    return status - 1;
  }

  std::cout << std::endl << "Readings from Pi Sense Hat sensors:\n";
  std::cout << "LPS25H:\n";
  std::cout << "Pressure P=" << lps25h_sensor->pressure_reading() << "[hPa]\n";
  std::cout << "Temperature T=" << lps25h_sensor->temperature_reading()
            << "[°C]\n\n";

  std::cout << "HTS221:\n";
  std::cout << "Relative Humidity R=" << hts221_sensor->humidity_reading()
            << "[%rH]\n";
  std::cout << "Temperature T=" << hts221_sensor->temperature_reading()
            << "[°C]\n\n";

  return status;
}
