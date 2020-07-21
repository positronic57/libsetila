/**
 * @file mcp3204.h
 *
 * @brief A header file from libsetila library. It defines MCP3204 class for
 * communication with Microchip MCP3204 ADC.
 *
 * @author Goce Boshkovski
 * @date 9 Aug 2019
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef MCP3204_H_
#define MCP3204_H_

#include "spi_slave_device.h"

#define START_BIT 0x04

/* Masks for selecting the input channels. */
#define MCP3204_CH_0 0x00
#define MCP3204_CH_1 0x40
#define MCP3204_CH_2 0x80
#define MCP3204_CH_3 0xC0

/** @brief SPI bus speed supported by MCP3204. */
#define MCP3204_SPI_BUS_SPEED		100000

/** @brief Size of data word of MCP3204(SPI bus parameter). */
#define MCP3204_SPI_BITS_PER_WORD	8

/**
 * \enum MCP3204_INPUT_CHANNEL_MODE
 * @brief Mode of operation for the MCP3204 input channels.
 */
enum class MCP3204_INPUT_CHANNEL_MODE: uint8_t
{
	SINGLE_ENDED = 0x02,//!< SINGLE_ENDED
	DIFFERENTIAL = 0xFB //!< DIFFERENTIAL
};

/**
 * \enum MCP3204_INPUT_CHANNEL
 * @brief Definition of MCP3204 input channels
 */
enum class MCP3204_INPUT_CHANNEL: uint8_t
{
	CH0,                                  	/**< Single ended channel 0 */
	CH1,                                  	/**< Single ended channel 1 */
	CH2,                                  	/**< Single ended channel 2 */
	CH3,                                  	/**< Single ended channel 3 */
	CH01,       /* CH0 = IN+, CH1 = IN- */	/**< Differential channel */
	CH10,       /* CH0 = IN-, CH1 = IN+ */	/**< Differential channel */
	CH23,       /* CH2 = IN+, CH3 = IN- */	/**< Differential channel */
	CH32        /* CH2 = IN-, CH3 = IN+ */	/**< Differential channel */
};

/**
 * \class MCP3204
 * @ingroup SPI_SLAVE_DEVICES
 *
 * @brief Class representing MCP3204 device.
 *
 * @example mcp3204_example.cpp
 */
class MCP3204: public SPI_Slave_Device
{
private:
	std::uint16_t m_digital_value = 0x0000;		/**< The result of the analog to digital conversion will be stored here. */
	float m_reference_voltage = 0.0;			/**< Reference voltage value. Required for representing the digital reading as analog value. */

public:
	/**
	 * @brief Class constructor.
	 */
	MCP3204() = default;

	/**
	 * @brief Class constructor.
	 *
	 * @param reference_voltage
	 */
	explicit MCP3204(float reference_voltage):
		SPI_Slave_Device(),
		m_digital_value(0),
		m_reference_voltage(reference_voltage)
	{};

	/**
	 * @brief Class destructor.
	 */
	~MCP3204() {};

	/**
	 * @brief Prepare the master for the communication with MCP3204.
	 * Set the reference voltage for internal conversion from digital to
	 * an analog value.
	 *
	 * @param[in] mcp3204_SPI_mode SPI mode supported by MCP3204
	 * @param[in] ref_voltage reference voltage value
	 *
	 * @return 0 in case of success, error code in case of a failure
	 */
	int configure(SPI_BUS_MODE mcp3204_SPI_mode, float ref_voltage);

	/**
	 * @brief Ask MCP3204 to do a analog-digital conversion and read the result.
	 *
	 * @param[in] input_channel selects the input channel
	 * @param[in] input_channel_mode set selects the channel input mode
	 *
	 * @return 0 in case there is no error in communication with MCP3204, error code if opposite
	 */
	int convert(MCP3204_INPUT_CHANNEL input_channel, MCP3204_INPUT_CHANNEL_MODE input_channel_mode);

	/**
	 * @brief Get the digital representation of the analog input.
	 * convert() function must be called before it.
	 *
	 * @return result of the analog to digital conversion
	 */
	uint16_t digital_value() const { return m_digital_value; };

	/**
	 * @brief Calculate the value of the analog input equivalent to the digital reading from MCP3204.
	 *
	 * @return analog equivalent to the digital reading
	 */
	float analog_value() {  return (m_digital_value * m_reference_voltage) / 4096; };
};


#endif /* MCP3204_H_ */
