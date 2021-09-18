/**
 * @file HTS221.h
 *
 * @brief A header file from libsetila library. It defines HTS221 class for communication with HT221 sensor.
 *
 * @author Goce Boshkovski
 * @date 01 May 2016
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef HTS221_H_
#define HTS221_H_

#include <stdint.h>

#include "i2c_slave_device.h"
#include "ST_sensor.h"

#define LTS221_ID	0xBC

/** \defgroup HTS221_DESC HTS221
 * @ingroup DEV_REG_CMD
 */

/** \defgroup HTS221_REG_DEF HTS221 Register Address Map
 * @ingroup HTS221_DESC
 */
/* @{ */
/** @brief WHO_AM_I register address. */
#define HTS221_WHO_AM_I 0x0F
/** @brief AV_CONF register address. */
#define HTS221_AV_CONF 0x10
/** @brief CTRL_REG1 register address. */
#define HTS221_CTRL_REG1 0x20
/** @brief CTRL_REG2 register address. */
#define HTS221_CTRL_REG2 0x21
/** @brief CTRL_REG3 register address. */
#define HTS221_CTRL_REG3 0x22
/** @brief STATUS_REG register address. */
#define HTS221_STATUS_REG 0x27
/** @brief HUMIDITY_OUT_L register address. */
#define HTS221_HUMIDITY_OUT_L 0x28
/** @brief HUMIDITY_OUT_H register address. */
#define HTS221_HUMIDITY_OUT_H 0x29
/** @brief TEMP_OUT_L register address. */
#define HTS221_TEMP_OUT_L 0x2A
/** @brief TEMP_OUT_H register address. */
#define HTS221_TEMP_OUT_H 0x2B
/** @brief CALIB_0 register address. */
#define HTS221_CALIB_0 0x30
#define H0_rH_x2 0
/** @brief CALIB_1 register address. */
#define HTS221_CALIB_1 0x31
#define H1_rH_x2 1
/** @brief CALIB_2 register address. */
#define HTS221_CALIB_2 0x32
#define T0_degC_x8 2
/** @brief CALIB_3 register address. */
#define HTS221_CALIB_3 0x33
#define T1_degC_x8 3
/** @brief CALIB_4 register address. */
#define HTS221_CALIB_4 0x34
/** @brief CALIB_5 register address. */
#define HTS221_CALIB_5 0x35
#define T1_T0_msb 5
/** @brief CALIB_6 register address. */
#define HTS221_CALIB_6 0x36
#define H0_T0_OUT_L 6
/** @brief CALIB_7 register address. */
#define HTS221_CALIB_7 0x37
#define H0_T0_OUT_H 7
/** @brief CALIB_8 register address. */
#define HTS221_CALIB_8 0x38
/** @brief CALIB_9 register address. */
#define HTS221_CALIB_9 0x39
/** @brief CALIB_A register address. */
#define HTS221_CALIB_A 0x3A
#define H1_T0_OUT_L 10
/** @brief CALIB_B register address. */
#define HTS221_CALIB_B 0x3B
#define H1_T0_OUT_H 11
/** @brief CALIB_C register address. */
#define HTS221_CALIB_C 0x3C
#define T0_OUT_L 12
/** @brief CALIB_D register address. */
#define HTS221_CALIB_D 0x3D
#define T0_OUT_H 13
/** @brief CALIB_E register address. */
#define HTS221_CALIB_E 0x3E
#define T1_OUT_L 14
/** @brief CALIB_F register address. */
#define HTS221_CALIB_F 0x3F
#define T1_OUT_H 15
/* @} */

/** \defgroup HTS221_AV_CONF_REG_DESC HTS221 AV_CONF register description
 * @ingroup HTS221_DESC
 */
/* @{ */
/** @brief AV_CONF_AVGT2 bit of AV_CONF register. */
#define HTS221_AV_CONF_AVGT2 0x05
/** @brief AV_CONF_AVGT1 bit of AV_CONF register. */
#define HTS221_AV_CONF_AVGT1 0x04
/** @brief AV_CONF_AVGT0 bit of AV_CONF register. */
#define HTS221_AV_CONF_AVGT0 0x03
/** @brief AV_CONF_AVGH2 bit of AV_CONF register. */
#define HTS221_AV_CONF_AVGH2 0x02
/** @brief AV_CONF_AVGH1 bit of AV_CONF register. */
#define HTS221_AV_CONF_AVGH1 0x01
/** @brief AV_CONF_AVGT0 bit of AV_CONF register. */
#define HTS221_AV_CONF_AVGH0 0x00
/* @} */

/** \defgroup HTS221_CTRL_REG1_REG_DESC HTS221 CTRL_REG1 register description
 * @ingroup HTS221_DESC
 */
/* @{ */
/** @brief PD bit of CTRL_REG1 register. */
#define HTS221_CTRL_REG1_PD 0x07
/** @brief BDU bit of CTRL_REG1 register. */
#define HTS221_CTRL_REG1_BDU 0x02
/** @brief ODR1 bit of CTRL_REG1 register. */
#define HTS221_CTRL_REG1_ODR1 0x01
/** @brief ODR0 bit of CTRL_REG1 register. */
#define HTS221_CTRL_REG1_ODR0 0x00
/* @} */

/** \defgroup HTS221_CTRL_REG2_REG_DESC HTS221 CTRL_REG2 register description
 * @ingroup HTS221_DESC
 */
/* @{ */
/** @brief BOOT bit of CTRL_REG2 register. */
#define HTS221_CTRL_REG2_BOOT 0x07
/** @brief HEATER bit of CTRL_REG2 register. */
#define HTS221_CTRL_REG2_HEATER 0x01
/** @brief ONE_SHOT bit of CTRL_REG2 register. */
#define HTS221_CTRL_REG2_ONE_SHOT 0x00
/* @} */

/** \defgroup HTS221_CTRL_REG3_REG_DESC HTS221 CTRL_REG3 register description
 * @ingroup HTS221_DESC
 */
/* @{ */
/** @brief DRDY_H_L bit of CTRL_REG3 register. */
#define HTS221_CTRL_REG3_DRDY_H_L 0x07
/** @brief PP_OD bit of CTRL_REG3 register. */
#define HTS221_CTRL_REG3_PP_OD 0x06
/** @brief DRDY bit of CTRL_REG3 register. */
#define HTS221_CTRL_REG3_DRDY 0x01
/* @} */

/** \defgroup HTS221_STATUS_REG_REG_DESC HTS221 STATUS_REG register description
 * @ingroup HTS221_DESC
 */
/* @{ */
/** @brief H_DA bit of STATUS_REG register. */
#define HTS221_STATUS_REG_H_DA 0x01
/** @brief T_DA bit of STATUS_REG register. */
#define HTS221_STATUS_REG_T_DA 0x00
/* @} */

/** \defgroup HTS221_DEFAULT_REG_VALUE HTS221 Default values of the registers
 * @ingroup HTS221_DESC
 */
/* @{ */
#define HTS221_AV_CONF_DEFAULT_VALUE 0x1B
#define HTS221_CTRL_REG1_DEFAULT_VALUE 0x00
#define HTS221_CTRL_REG2_DEFAULT_VALUE 0x00
#define HTS221_CTRL_REG3_DEFAULT_VALUE 0x00
#define HTS221_STATUS_REG_DEFAULT_VALUE 0x00
/* @} */

/**
 * \class HTS221
 * @ingroup I2C_SLAVE_DEVICES
 *
 * @brief A class derived from I2CSensor class that describes HTS221 sensor.
 *
 * @example pi_sense_hat.cpp
 */
class HTS221: public ST_Sensor//I2C_Slave_Device
{
private:
	bool m_calibration_table_read = false;				/**< A flag for reading the calibration table. */
	bool m_device_id_verified = false;					/**< A flag for verification of the sensor type with the help of WHO_AM_I register. */
	float m_temperature_reading = 0.0;					/**< A sensor temperature reading.*/
	float m_humidity_reading = 0.0;						/**< A sensor humidity reading. */
	uint8_t m_calibration_table[16] = { 0x00 };			/**< HTS221 calibration table. */
	uint8_t m_humidity_out[2] = { 0x00 };				/**< A humidity buffer. */
	uint8_t m_temperature_out[2] = { 0x00 };			/**< A temperature buffer. */
	uint8_t m_CTRL_REG1 = { 0x00 };						/**< Holds CTRL_REG1 value between power ON and OFF requests. */
	uint8_t m_CTRL_REG2 = { 0x00 };						/**< Holds CTRL_REG2 value between power ON and OFF requests. */

public:
	/**
	 *
	 * @param[in] interface_type defines the sensor data communication interface (SPI or I2C)
	 * @param[in] bus_master_device pointer to the object that represents the master of the SPI/I2C bus where the sensor is connected to
	 * @param[in] I2C_slave_address I2C address of the sensor in case the I2C communication
	 */
	explicit HTS221(Slave_Device_Type interface_type, Bus_Master_Device *bus_master_device, uint8_t I2C_slave_address):
	ST_Sensor(interface_type, bus_master_device, I2C_slave_address)
	{};

	/**
	 * @brief A destructor of the class.
	 */
	~HTS221() { }

	virtual int set_mode_of_operation(mode_of_operation_t mode_of_operation, output_data_rate_t output_data_rate = ST_Sensor::output_data_rate_t::ODR_ONE_SHOT) override;
	virtual int set_resolution(uint8_t average_1, uint8_t average_2 = 0x00) override;
	virtual int get_sensor_readings() override;
	virtual int verify_device_id() override;

	/**
	 * @brief Provides the value of the sensor temperature reading.
	 *
	 * @return float returns the value of the class member temperatureReading.
	 */
	float temperature_reading() const { return m_temperature_reading; };

	/**
	 * @brief Provides the value of the sensor humidity reading.
	 *
	 * @return float returns the value of the class member humidityReading.
	 */
	float humidity_reading() const { return m_humidity_reading; };

private:
	/**
	 * @brief Calculates the relative humidity bases on the sensor reading and place the result in the class member ""m_humidity_reading.
	 *
	 * @return int always returns 0.
	 */
	void calculate_realtive_humidity();

	/**
	 * @brief Calculates the temperature bases on the sensor reading and place the result in the class member "m_temperature_reading".
	 *
	 * @return int always returns 0.
	 */
	int calculate_temperature();

	/**
	 * @brief Reads the content of the calibration table from HTS221 sensor.
	 *
	 * The start address of the calibration table is HTS221_CALB_0.
	 * The MSB bit of the register address is set to 1 for enabling address auto-increment.
	 *
	 * @return int returns an error code in case there is a failure in communication with the sensor, 0 for successful read.
	 */
	int read_calibration_table();

	/**
	 *
	 * @param output_data_rate
	 * @return
	 */
	int config_continuous_mode(output_data_rate_t output_data_rate);

	/**
	 * @brief Switch the sensor into a power-down mode by setting PD bit of CTRL_REG1 to 0.
	 *
	 * @return int an error code in case there is a failure in the communication with the sensor, 0 for success.
	 */
	int power_down(void);

	/**
	 * @brief Initiate the measurement process for humidity and temperature.
	 *
	 * The function starts a single acquisition of temperature and humidity by setting the ONE_SHOT bit of CTRL_REG2 register.
	 * After that get_sensor_readings() function is called for reading the measurement outputs.
	 *
	 * @return int returns an error code in case there is a failure in communication with the sensor; ERROR_SENSOR_READING_TIME_OUT when there is no measurement outputs
	 * available SENSOR_READING_WATCHDOG_COUNTER number of reading attempts; 0 for successful measurement cycle.
	 */
	int do_one_shot_measurement(void);
};

#endif /* HTS221_H_ */
