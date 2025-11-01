/**
 * @file bmp085_bmp180.cpp
 *
 * @brief Example using libsetila for measuring pressure and temperature with
 * Bosh BMP085/BMP180 sensor. The sensor readings are first stored in FIFO_queue data
 * container and after all the readings are gatherd, the content of the queue is printed
 * on the standard output.
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 */

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>

#include "setila/BMP085.h"
#include "setila/bdc/FIFO_queue.h"

struct BMP085_readings {
  double temperature;
  double pressure;
  int number;
};

int main() {

  constexpr uint8_t BMP085_I2C_Address{0x77};
  constexpr size_t number_of_samples{30};

  // Example tested on Pi Zero W rev 1.1 were I2C interface is /dev/i2c-1.
  // Put the correct device name for your platform
  std::unique_ptr<Bus_Master_Device> i2c_bus_master{
      new Bus_Master_Device("/dev/i2c-1", BUS_TYPE::I2C_BUS)};

  std::unique_ptr<BMP085> bmp085_sensor{new BMP085(BMP085_I2C_Address)};

  FIFO_queue<struct BMP085_readings> measurement_buffer(number_of_samples);

  struct BMP085_readings reading;

  int meas_count{0};

  if (i2c_bus_master->open_bus() < 0) {
    std::cout << "Failed to open master bus\n";
    return -1;
  }

  if (bmp085_sensor->attach_to_bus(i2c_bus_master.get()) ||
      bmp085_sensor->init_sensor()) {
    std::cout << "BMP085 sensor initialization failed.\n";
    return -1;
  }

  std::cout << "Collecting " << number_of_samples << " sensor readings for pressure and temperature\n";
  while (!measurement_buffer.isFull()) {
    if (bmp085_sensor->measure_temperature_pressure()) {
      std::cout << "Failed to retrieve sensor data.\n";
      return -1;
    }

    reading.pressure = bmp085_sensor->pressure_reading();
    reading.temperature = bmp085_sensor->temperature_reading();
    reading.number = ++meas_count;

    measurement_buffer.addElement(reading);

    std::cout << "Measurement #" << meas_count << '\n';

    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  std::cout << "BMP085 readings:\n";

  std::cout << std::showpos;
  while (!measurement_buffer.isEmpty()) {
    measurement_buffer.getElement(reading);

    std::cout << "Reading #" << reading.number << '\n';
    std::cout << "Temperature: t = " << reading.temperature << "°C\n";
    std::cout << "Pressure: P = " << reading.pressure << "hPa\n\n";
  }

  return 0;
}
