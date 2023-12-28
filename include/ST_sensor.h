/**
 * @file ST_sensor.h
 *
 * @date Apr 28, 2021
 * @author Goce Boshkovski
 *
 * @brief Definition of ST_Sensor class.
 *
 * @copyright GNU General Public License v3
 */

#ifndef INCLUDE_ST_SENSOR_H_
#define INCLUDE_ST_SENSOR_H_

#include "slave_device.h"

/**
 * \class ST_Sensor
 *
 * @brief Base class for ST sensors with I2C and/or SPI interface.
 *
 * The class defines few virtual function for high level abstraction of the
 * configuration process, measurement commands and reading of the sensor data.
 * Lower level configuration functions and registers R/W operation, specific
 * for the targeted sensor must be defined and implemented additionally in
 * the child classes derived from this base class.
 *
 * The data communication interface is defined as an instance of the Slave_Device class
 * and can cover both I2C and SPI.
 *
 * @example LPS22HB.h LPS22HB support using ST_Sensor as a base class
 *
 */
class ST_Sensor
{
private:
	Slave_Device_Type m_interface_type = Slave_Device_Type::I2C_SLAVE_DEVICE; /**< Type of the communication interface (I2C/SPI). */

	ST_Sensor() {};

public:

	/**
	 * @brief One of the possible mode of operation found in ST sensors.
	 */
	enum MODE_OF_OPERATION {
		OP_POWER_DOWN = 0,	/**< Power down mode. */
		OP_ONE_SHOT,					/**< One shot (on demand) measurement. */
		OP_CONTINUOUS,				/**< Continuous measurement with defined output data rate. */
		OP_NORMAL_MODE,			/**< Same as continuous mode. */
		OP_SLEEP_MODE,
		OP_FIFO_MODE,					/**< FIFO mode of operation. */
		OP_FIFO_MEAN_MODE	/**< FIFO MEAN mode of operation. */
	} m_mode_of_operation = OP_POWER_DOWN;

	enum FIFO_TYPE {
		FIFO_DISABLED = 0,
		BYPASS_MODE,
		FIFO,
		STREAM,
		STREAM_TO_FIFO,
		BYPASS_TO_FIFO
	} m_fifo_type;

	/**
	 * @brief List of possible output data rates found in different ST sensors.
	 */
	enum OUTPUT_DATA_RATE {
		ODR_ONE_SHOT = 0,	/**< Measurement only on demand */
		ODR_1_Hz,        	/**< Output data rate of 1Hz */
		ODR_7_Hz,        	/**< Output data rate of 7Hz */
		ODR_10_Hz,       	/**< Output data rate of 10Hz */
		ODR_12_5_Hz,     	/**< Output data rate of 12.5Hz */
		ODR_25_Hz,       	/**< Output data rate of 25Hz */
		ODR_50_Hz,       	/**< Output data rate of 50Hz */
		ODR_75_Hz,        	/**< Output data rate of 75Hz */
		ODR_95_Hz,			/**< Output data rate of 95Hz */
		ODR_190_Hz,			/**< Output data rate of 190Hz */
		ODR_380_Hz,			/**< Output data rate of 380Hz */
		ODR_760_Hz			/**< Output data rate of 760Hz */
	} m_output_data_rate = ODR_1_Hz;

	enum FULL_SCALE {
		FS_250_DPS = 0,
		FS_500_DPS,
		FS_2000_DPS
	};
#if 0
	using mode_of_operation_t = modes_of_operation_enum;
	using output_data_rate_t = output_data_rate_enum;
	using fifo_type_t = fifo_type_enum;
	using full_scale_t = full_scale_enum;
#endif
	Slave_Device *m_interface = nullptr;	/**< Represents the sensor communication interface. It can be an instance of I2C_Slave_Device or SPI_Slave_Deice class depening of the m_interface_type value. */

	/**
	 * @brief Constructor
	 * Some ST sensors have hardware support for I2C or SPI only interface or they support both of them.
	 * The constructor defines which interface type will be used by the sensor for data exchange.
	 * As slave device, the sensor must be attached to a bus controlled by the master device.
	 * When connected to I2C bus, the sensor requires I2C address.
	 *
	 * @param[in] interface_type Slave_Device_Type selects the data interface type
	 * @param[in] bus_master_device pointer to the instance of the Bus_Master_Device class
	 * @param[in] i2c_address I2C address in case of I2C interface, ignored for SPI and set to default value of 0x03 if not specified
	 */
	explicit ST_Sensor(Slave_Device_Type interface_type, Bus_Master_Device *bus_master_device, uint8_t i2c_address = 0x03);

	virtual ~ST_Sensor();

	Slave_Device_Type interface_type() { return m_interface_type; };
#if 0
	mode_of_operation_t mode_of_operation() { return m_mode_of_operation; };
	output_data_rate_t output_data_rate() { return m_output_data_rate; };
#endif
	/**
	 * @brief Check the type/model of the sensor by reading
	 * the value of WHO_AM_I register from ST sensor and comparing it with
	 * the known value for that sensor model.
	 *
	 * @param[in] who_am_I_reg_addr address of the WHO_AM_I register of the ST Sensor
	 * @param[in] id_value expected value of the WHO_AM_I register for the sensor
	 * @return int 0 if the verification was successful, ERROR_WRONG_DEVICE_MODEL in case the value of the WHO_AM_I register does not match, ERROR_READ_FAILED for failed
	 * attempt to read the WHO_AM_I register
	 */
	int verify_device_id(uint8_t who_am_I_reg_addr, uint8_t id_value);

	/**
	 * @brief High level function for configuring the sensor. It sets the mode of operation and the output data rate by writing
	 * values in the sensors control registers based on the given arguments.
	 * The registers and their values for achieving the desired sensor configuration are individual for each ST sensor type.
	 * The logic will be implemented in the child class that represents the target ST sensor.
	 *
	 * @param[in] mode_of_operation predefined enum for the modes of operation found in ST sensors
	 * @param[in] output_data_rate predefined enum for the output data data rates found in ST sensors
	 *
	 * @return int 0 in case of successful configuration write, appropriate error code otherwise
	 */
	virtual int set_mode_of_operation(MODE_OF_OPERATION mode_of_operation, OUTPUT_DATA_RATE output_data_rate = ODR_ONE_SHOT) = 0;

	virtual int set_mode_of_operation(
		OUTPUT_DATA_RATE output_data_rate = ST_Sensor::OUTPUT_DATA_RATE::ODR_95_Hz,
		FULL_SCALE full_scale = ST_Sensor::FULL_SCALE::FS_250_DPS,
		MODE_OF_OPERATION mode_of_operation =ST_Sensor:: MODE_OF_OPERATION::OP_NORMAL_MODE,
		FIFO_TYPE  fifo_type = ST_Sensor::FIFO_TYPE::FIFO_DISABLED
	)  = 0;

	/**
	 * @brief Set the resolution for the physical quantity(ies) measured by the sensor.
	 * Different ST sensors support different measurement resolution(s). The registers used for that purpose
	 * varies from sensor to sensor. The functionality must be implemented in the child class following the correct way
	 * for the targeted ST sensor.
	 *
	 * @param[in] average_1 bit coded number of averaged samples for the first physical quantity
	 * @param[in] average_2 bit coded number of averaged samples for the second physical quantity if any
	 *
	 * @return int 0 for success, error code otherwise
	 */
	virtual int set_resolution(uint8_t average_1, uint8_t average_2 = 0x00) = 0;

	/**
	 * @brief High level function for reading the values of the data registers for the
	 * measured physical quantities. Based on the defined mode of operation, the actual data reading can be preceded
	 * with a command for measurement start. As other two functions, this procedure is also a sensor dependent.
	 *
	 * @return int 0 for success, error code otherwise
	 */
	virtual int get_sensor_readings() = 0;

};


#endif /* INCLUDE_ST_SENSOR_H_ */
