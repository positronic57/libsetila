/**
 * @file LPS22HB.h
 *
 * @brief A header file from libsetila library. It defines LPS22HB class for communication with LPS22HB sensor.
 *
 * @author Goce Boshkovski
 * @date 17.04.2021
 *
 * @copyright GNU General Public License v3
 *
 * Supported LPS22HB modes of operation:
 * - ONE SHOT.
 *
 * Only a measurement of absolute temperature and pressure values is supported.
 *
 * Typical use case:
 *
 * 1. Call the constructor with LPS22HB I2C address, communication interface type and pointer to the bus master object. For example 0x5C:
 *
 *  LPS22HB(Slave_Device_Type::I2C_SLAVE_DEVICE, i2c_bus_master, 0x5C);
 *
 * 2. Assign ONE SHOT as mode of operation
 *
 *  set_mode_of_operation(ST_Sensor::mode_of_operation_t::OP_ONE_SHOT);
 *
 * 3. Perform a measurement and get the pressure and temperature readings
 *
 *  get_sensor_readings();
 *
 * @example arduino_mkr_env_shield_rev2.cpp
 *
 */

#ifndef LPS22HB_H_
#define LPS22HB_H_

#if 0
#include "i2c_slave_device.h"
#else
#include "ST_sensor.h"
#endif

/** @brief Value of the WHO_AM_I register. */
#define LPS22HB_ID 0xB1

/** @brief Value for enabling the power down/single shot mode of operation. */
#define LPS22HB_POWER_DOWN_MODE_DEF	0x0F

/** @brief PLS22HB I2C address when SA0 pad is contected to ground. */
#define LPS22HB_ADDR_SA0_0	0x5C	//If the SA0 pad of LPS22HB is connected to ground

/** @brief PLS22HB I2C address when SA0 pad is connected to voltage supply. */
#define LPS22HB_ADDR_SA0_1	0x5D	//If the SA0 pad of LPS22HB is connected to voltage supply

/** \defgroup LPS22HB_DESC LPS22HB
 * @ingroup DEV_REG_CMD
 */

/** \defgroup LPS22HB_REG_DEF LPS22HB Register address map
 * @ingroup LPS22HB_DESC
 */
/* @{ */
/** @brief INT_CFG register address. */
#define LPS22HB_INT_CFG 0x0B

/** @brief THS_P_L register address. */
#define LPS22HB_THS_P_L 0x0C

/** @brief THS_P_H register address. */
#define LPS22HB_THS_P_H 0x0D

/** @brief WHO_AM_I register address. */
#define LPS22HB_WHO_AM_I 0x0F

/** @brief CTRL_REG1 register address. */
#define LPS22HB_CTRL_REG1 0x10

/** @brief CTRL_REG2 register address. */
#define LPS22HB_CTRL_REG2 0x11

/** @brief CTRL_REG3 register address. */
#define LPS22HB_CTRL_REG3 0x12

/** @brief FIFO_CTRL register address. */
#define LPS22HB_FIFO_CTRL 0x14

/** @brief REF_P_XL register address. */
#define LPS22HB_REF_P_XL 0x15

/** @brief REF_P_XL register address. */
#define LPS22HB_REF_P_L 0x16

/** @brief REF_P_H register address. */
#define LPS22HB_REF_P_H 0x17

/** @brief RPDS_L register address. */
#define LPS22HB_RPDS_L 0x18

/** @brief RPDS_H register address. */
#define LPS22HB_RPDS_H 0x19

/** @brief RES_CONF register address. */
#define LPS22HB_RES_CONF 0x1A

/** @brief INT_SOURCE register address. */
#define LPS22HB_INT_SOURCE 0x25

/** @brief FIFO_STATUS register address. */
#define LPS22HB_FIFO_STATUS 0x26

/** @brief STATUS_REG register address. */
#define LPS22HB_STATUS_REG 0x27

/** @brief POUT_XL register address. */
#define LPS22HB_PRESS_OUT_XL 0x28

/** @brief PRESS_OUT_L register address. */
#define LPS22HB_PRESS_OUT_L 0x29

/** @brief PRESS_OUT_H register address. */
#define LPS22HB_PRESS_OUT_H 0x2A

/** @brief TEMP_OUT_L register address. */
#define LPS22HB_TEMP_OUT_L 0x2B

/** @brief TEMP_OUT_H register address. */
#define LPS22HB_TEMP_OUT_H 0x2C

/** @brief Filter reset register address. */
#define LPS22HB_LPFP_RES 0x33


/* @} */

/** \defgroup LPS22HB_CTRL_REG1_REG_DESC LPS22HB CTRL_REG1 register description
 * @ingroup LPS22HB_DESC
 */
/* @{ */
/** @brief Output data rate selection, ODR2 bit of CTRL_REG1 register. */
#define LPS22HB_CTRL_REG1_ODR2 0x06

/** @brief Output data rate selection, ODR1 bit of CTRL_REG1 register. */
#define LPS22HB_CTRL_REG1_ODR1 0x05

/** @brief Output data rate selection, ODR0 bit of CTRL_REG1 register. */
#define LPS22HB_CTRL_REG1_ODR0 0x04

/** @brief Enable low-pass filter on pressure data when Continuous mode is used, EN_LPFP bit of CTRL_REG1 register. */
#define LPS22HB_CTRL_REG1_EN_LPFP 0x03

/** @brief Low-pass configuration register, LPFP_CFG bit of CTRL_REG1 register. */
#define LPS22HB_CTRL_REG1_LPFP_CFG 0x02

/** @brief Block data update, BDU bit of CTRL_REG1 register. */
#define LPS22HB_CTRL_REG1_BDU 0x01

/** @brief SPI Serial Interface Mode selection, SIM bit of CTRL_REG1 register. */
#define LPS22HB_CTRL_REG1_SIM 0x00
/* @} */

/** \defgroup LPS22HB_CTRL_REG2_REG_DESC LPS22HB CTRL_REG2 register description
 * @ingroup LPS22HB_DESC
 */
/* @{ */
/** @brief Reboot memory content, BOOT bit of CTRL_REG2 register. */
#define LPS22HB_CTRL_REG2_BOOT 0x07

/** @brief FIFO enable, FIFO_EN bit of CTRL_REG2 register. */
#define LPS22HB_CTRL_REG2_FIFO_EN 0x06

/** @brief Stop on FIFO watermark, STOP_ON_FTH bit of CTRL_REG2 register. */
#define LPS22HB_CTRL_REG2_STOP_ON_FTH 0x05

/** @brief Register address automatically incremented during a multiple byte access with a
serial interface (I2C or SPI), IF_ADD_INC bit of CTRL_REG2 register. */
#define LPS22HB_CTRL_REG2_IF_ADD_INC 0x04

/** @brief Disable I2C interface, I2C_DIS bit of CTRL_REG2 register. */
#define LPS22HB_CTRL_REG2_I2C_DIS 0x03

/** @brief Software reset, SWRESET bit of CTRL_REG2 register. */
#define LPS22HB_CTRL_REG2_SWRESET 0x02

/** @brief One-shot enable, ONE_SHOT bit of CTRL_REG2 register. */
#define LPS22HB_CTRL_REG2_ONE_SHOT 0x00
/* @} */

/** \defgroup LPS22HB_CTRL_REG3_REG_DESC LPS22HB CTRL_REG3 register description
 * @ingroup LPS22HB_DESC
 */
/* @{ */
/** @brief Interrupt active-high/low bit INT_H_L of CTRL_REG3 register. */
#define LPS22HB_CTRL_REG3_INT_H_L 0x07

/** @brief Push-pull/open drain selection on interrupt pads PP_OD bit of CTRL_REG3 register. */
#define LPS22HB_CTRL_REG3_PP_OD 0x06

/** @brief FIFO full flag on INT_DRDY pin F_FSS5 bit of CTRL_REG3 register. */
#define LPS22HB_CTRL_REG3_F_FSS5 0x05

/** @brief FIFO watermark status on INT_DRDY pin F_FTH bit of CTRL_REG3 register. */
#define LPS22HB_CTRL_REG3_F_FTH 0x04

/** @brief FIFO overrun interrupt on INT_DRDY pin F_OVR bit of CTRL_REG3 register. */
#define LPS22HB_CTRL_REG3_F_OVR 0x03

/** @brief Data-ready signal on INT_DRDY pin DRDY bit of CTRL_REG3 register. */
#define LPS22HB_CTRL_REG3_DRDY 0x02

/** @brief Data signal on INT_DRDY pin control S2 bit of CTRL_REG3 register. */
#define LPS22HB_CTRL_REG3_INT_S2 0x01

/** @brief Data signal on INT_DRDY pin control S1 bit of CTRL_REG3 register. */
#define LPS22HB_CTRL_REG3_INT_S1 0x00
/* @} */

/** \defgroup LPS22HB_STATUS_REG_REG_DESC LPS22HB STATUS_REG register description
 * @ingroup LPS22HB_DESC
 */
/* @{ */
/** @brief Temperature data overrun T_OR bit of STATUS_REG register. */
#define LPS22HB_STATUS_REG_T_OR 0x05

/** @brief Pressure data overrun P_OR bit of STATUS_REG register. */
#define LPS22HB_STATUS_REG_P_OR 0x04

/** @brief Pressure data available P_DA bit of STATUS_REG register. */
#define LPS22HB_STATUS_REG_T_DA 0x01

/** @brief Temperature data T_DA bit of STATUS_REG register. */
#define LPS22HB_STATUS_REG_P_DA 0x00
/* @} */

/** \defgroup LPS22HB_RES_CONF_REG_DESC LPS22HB RES_CONF register description
 * @ingroup LPS22HB_DESC
 */
/* @{ */
/** @brief Low current mode enable LC_EN bit of RES_CONF register. */
#define LPS22HB_RES_CONF_LC_EN 0x00
/* @} */

/** \defgroup LPS22HB_FIFO_CTRL_REG_DESC LPS22HB FIFO_CTRL register description
 * @ingroup LPS22HB_DESC
 */
/* @{ */
/** @brief F_MODE2 bit of FIFO_CTRL register. */
#define FIFO_CTRL_F_MODE2 0x07

/** @brief F_MODE1 bit of FIFO_CTRL register. */
#define FIFO_CTRL_F_MODE1 0x06

/** @brief F_MODE0 bit of FIFO_CTRL register. */
#define FIFO_CTRL_F_MODE0 0x05

#define FIFO_CTRL_WTM4	0x04

#define FIFO_CTRL_WTM3	0x03

#define FIFO_CTRL_WTM2	0x02

#define FIFO_CTRL_WTM1	0x01

#define FIFO_CTRL_WTMO	0x00
/* @} */

/* @} */


/** 
 * \class LPS22HB
 *  @ingroup I2C_SLAVE_DEVICES
 *
 *  @brief A class derived from I2CSensor class
 *  that describes LPS22HB sensor.
 *
 *  @example arduino_mkr_env_shield_rev2.cpp
 */
class LPS22HB: public ST_Sensor
{
private:
	bool m_device_id_verified = false;		/**< A flag for verification of the sensor type with the help of WHO_AM_I register. */
	float m_pressure_reading = 0.0;			/**<  The last pressure value measured by the sensor. */
	float m_temperature_reading = 0.0;		/**<  The last temperature value measured by the sensor. */
	uint8_t m_CTRL_REG1 = 0x00;				/**< Holds the last value of the CTRL_REG1. */
	uint8_t m_CTRL_REG2 = 0x10;				/**< Holds the last value of the CTRL_REG2. */
	uint8_t m_CTRL_REG3 = 0x00;				/**< Holds the last value of the CTRL_REG3. */

	LPS22HB() = default;

public:
	/**
	 * @brief A constructor.
	 *
	 * @param[in] interface_type enum for selecting the communication interface of the sensor (I2C or SPI)
	 * @param[in] bus_master_device pointer to the object that represents the master of the bus where the sensor is connected
	 * @param[in] I2C_slave_address an I2C address of the sensor
	 */
	explicit LPS22HB(Slave_Device_Type interface_type, Bus_Master_Device *bus_master_device, uint8_t I2C_slave_address):
	ST_Sensor(interface_type, bus_master_device, I2C_slave_address)
	{};

	/**
	 * @brief A destructor of the class.
	 */
	~LPS22HB() {};

	/**
	 * @brief A low level function for custom sensor configuration using the control registers of the sensor.
	 *
	 * Can be used when the higher level function like set_mode_of_operation() is not enough for the
	 * application. It will change the values of the CTRL_REG1, CTRL_REG2 and CTRL_REG3 registers.
	 *
	 * @param[in] CTRL_REG1_value the new value of the CTRL_REG1 register.
	 * @param[in] CTRL_REG2_value the new value of the CTRL_REG2 register.
	 * @param[in] CTRL_REG3_value the new value of the CTRL_REG3 register.
	 *
	 * @return int returns 0 in case of successful init, ERROR_READ/WRITE_FAILED code is case of failure, or
	 * ERROR_WRONG_DEVICE_MODEL when the target device is not recognized as LPS22HB.s
	 */
	int custom_config(uint8_t &CTRL_REG1_value, uint8_t &CTRL_REG2_value, uint8_t &CTRL_REG3_value);

	/**
	 * @brief Provides the last pressure reading in hPa.
	 *
	 * @return float Pressure value in hPa.
	 */
	float last_pressure_reading() const { return m_pressure_reading; };

	/**
	 * @brief Provides the last temperature reading in degrees C.
	 *
	 * @return float Temperature value in degrees C.
	 */
	float last_temperature_reading() const {  return m_temperature_reading; };

	virtual int set_mode_of_operation(mode_of_operation_t mode_of_operation, output_data_rate_t output_data_rate = ST_Sensor::output_data_rate_t::ODR_ONE_SHOT) override;
	virtual int set_resolution(uint8_t average_1, uint8_t average_2 = 0x00) override;
	virtual int get_sensor_readings() override;
	virtual int verify_device_id() override;

private:
	/**
	 * @brief Puts the device is in power-down mode. Only one shot measurement is possible.
	 *
	 * @return 0 in case the new CTRL_REG1 value is sent to the sensor successfully, error code ERROR_WRITE_FAILED
	 * when there is an issue with sending data to the sensor.
	 */
	int enable_one_shot_mode(void);

	/**
	 * @brief Starts a one shot measurement by setting bit in CTRL_REG2 register.
	 *
	 * @return 0 new register value successfully sent, ERROR_WRITE_FAILED in case of an error
	 */
	int do_one_shot_measurement(void);

	/**
	 * @brief Reads the values of the output pressure and temperature registers.
	 * The reading is done in a loop, waiting for bits T_DA and P_DA from STATUS register to be set to 1.
	 * If this does not happened in SENSOR_READING_WATCHDOG_COUNTER number of STATUS register checks, the function returns
	 * with ERROR_SENSOR_READING_TIME_OUT code.
	 *
	 * The function can be used in case of ONE_SHOT and continuous measurements.
	 * @return 0 in case of success, error code in case of a R/W failure of reading time out.
	 */
	int read_data_registers();

	/**
	 * @brief Updates the values of the control register(s) for configuring the continuous mode and
	 * defines the output data rate of the sensor.
	 *
	 * @param[in] output_data_rate the new output data rate supported by the sensor
	 *
	 * @return 0 in case of success, ERROR_WRITE_FAILED in case of an failure
	 */
	int config_continuous_mode(output_data_rate_t output_data_rate);
};

#endif /* LPS22HB_H_ */
