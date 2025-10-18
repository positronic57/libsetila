/**
 * @file bmp085_bmp180.cpp
 *
 * @brief Example using libsetila for measuring pressure and temperature with
 * Bosh BMP085/BMP180 sensor. The sensor readings are stored in FIFO_queue data
 * container.
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 */

#include <cstddef>
#include <cstdint>
#include <iostream>

#include "setila/BMP085.h"
#include "setila/bdc/FIFO_queue.h"

extern "C" {
#include <unistd.h> //required for sleep()
};

struct BMP085_readings {
  double temperature;
  double pressure;
  int number;
};

int main() {

  constexpr uint8_t BMP085_I2C_Address{0x77};
  constexpr size_t number_of_samples{30};

  Bus_Master_Device *i2c_bus_master =
      new Bus_Master_Device("/dev/i2c-0", BUS_TYPE::I2C_BUS);

  BMP085 *bmp085_sensor = new BMP085(BMP085_I2C_Address);

  FIFO_queue<struct BMP085_readings> measurement_buffer(number_of_samples);

  struct BMP085_readings reading;

  int status{0};
  int meas_count{0};

  do {
    if (i2c_bus_master->open_bus() < 0) {
      std::cout << "Failed to open master bus\n";
      status = -1;
      break;
    }

    if (bmp085_sensor->attach_to_bus(i2c_bus_master) ||
        bmp085_sensor->init_sensor()) {
      status = -1;
      std::cout << "BMP085 sensor initialization failed.\n";
      break;
    }

    while (!measurement_buffer.isFull()) {
      if (bmp085_sensor->measure_temperature_pressure()) {
        std::cout << "Failed to retrieve sensor data.\n";
        status = -1;
        break;
      }

      reading.pressure = bmp085_sensor->pressure_reading();
      reading.temperature = bmp085_sensor->temperature_reading();
      reading.number = ++meas_count;

      measurement_buffer.addElement(reading);

      std::cout << "Measurement #" << meas_count << '\n';
      sleep(2);
    }

    if (status) {
      break;
    }

    std::cout << "BMP085 readings:\n";

    std::cout << std::showpos;
    while (!measurement_buffer.isEmpty()) {
      measurement_buffer.getElement(reading);

      std::cout << "Reading #" << reading.number << '\n';
      std::cout << "Temperature: t = " << reading.temperature << "°C\n";
      std::cout << "Pressure: P = " << reading.pressure << "hPa\n\n";
    }

  } while (0);

  delete bmp085_sensor;
  delete i2c_bus_master;

  return status;
}
