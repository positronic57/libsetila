/**
 * @file l3dg20_FIFO_example.cpp
 *
 * @brief Example for reading raw angluar rate values for X, Y and Z axes from
 * L3GD20 in FIFO mode of operation.
 *
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 *
 */
#include <iostream>
#include <thread>
#include <chrono>

#include "setila/setila_i2c.h"
#include "setila/L3GD20.h"

int main()
{
    int status = 0;

    Bus_Master_Device *i2c_bus_master = new Bus_Master_Device("/dev/i2c-1", BUS_TYPE::I2C_BUS);

    L3GD20 *L3GD20_sensor = new L3GD20(Slave_Device_Type::I2C_SLAVE_DEVICE, i2c_bus_master, 0x6B);

    if (i2c_bus_master->open_bus() < 0) {
	    std::cout << "Failed to open master bus" << std::endl;
	    return -1;
    }

    // Same as setting fields in L3GD20::Config structure and calling L3GD20::configure()
    status = L3GD20_sensor->set_mode_of_operation(
		  ST_Sensor::OUTPUT_DATA_RATE::ODR_95_Hz,
		  ST_Sensor::FULL_SCALE::FS_250_DPS,
		  ST_Sensor::MODE_OF_OPERATION::OP_NORMAL_MODE,
		  ST_Sensor::FIFO_TYPE::FIFO
	);
    if (status) {
        std::cout << "Error: configure sensor failed with error " << status << std::endl;
        return status;
    }

    // Read the content of the FIFO buffer of the sensor
    status = L3GD20_sensor->get_sensor_readings();
    if (status) {
        std::cout << "Error: get sensor data failed with error " << status << std::endl;
	return status;
    }

    std::cout << "Content of the FIFO, starting from the latest to the oldest measurement:" << std::endl;
    // Newest reading at the top of the FIFO
    std::cout 
           << " raw X = " << L3GD20_sensor->data.angular_rate_X
           << " Y = " << L3GD20_sensor->data.angular_rate_Y
           << " Z = " << L3GD20_sensor->data.angular_rate_Z
	   << " in [mdps] X = " << L3GD20::angular_rate_in_mdps(L3GD20_sensor->config, L3GD20_sensor->data.angular_rate_X)
	   << " Y = " << L3GD20::angular_rate_in_mdps(L3GD20_sensor->config, L3GD20_sensor->data.angular_rate_Y)
	   << " Z = " << L3GD20::angular_rate_in_mdps(L3GD20_sensor->config, L3GD20_sensor->data.angular_rate_Z)
           << std::endl;
    // The rest of the readings till the oldest sample
    for(int item = L3GD20_FIFO_SIZE_IN_RAW_VALUES - 4; item >= 2; item = item - 3) {
        std::cout  
           << " raw X = " << L3GD20_sensor->data.FIFO[item]
           << " Y = " << L3GD20_sensor->data.FIFO[item - 1]
           << " Z = " << L3GD20_sensor->data.FIFO[item - 2]
	   << " in [mdps] X = " << L3GD20::angular_rate_in_mdps(L3GD20_sensor->config, L3GD20_sensor->data.FIFO[item])
	   << " Y = " << L3GD20::angular_rate_in_mdps(L3GD20_sensor->config, L3GD20_sensor->data.FIFO[item - 1])
	   << " Z = " << L3GD20::angular_rate_in_mdps(L3GD20_sensor->config, L3GD20_sensor->data.FIFO[item - 2])
           << std::endl;
    }

    return status;
}


