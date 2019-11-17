/**
 * @file spi_bus_master.h
 *
 * @brief Defines the SPI_Bus_Master_Device class.
 *
 * @author Goce Boshkovski
 * @date 4 Aug 2019
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef SPI_BUS_MASTER_H_
#define SPI_BUS_MASTER_H_

#include <linux/spi/spidev.h>

#include "bus_master_device.h"
/**
 * \enum SPI_BUS_MODE
 * @brief Defines the SPI modes (combinations of polarity and phases of the bus signals).
 */
enum class SPI_BUS_MODE: uint8_t {
	MODE_0 = SPI_MODE_0,	/**< MODE_0 SPI Mode 0 */
	MODE_1 = SPI_MODE_1,	/**< MODE_1 SPI Mode 1 */
	MODE_2 = SPI_MODE_2,	/**< MODE_2 SPI Mode 2 */
	MODE_3 = SPI_MODE_3 	/**< MODE_3 SPI Mode 3 */
};

/**
 * \class SPI_Bus_Master_Device
 * @ingroup SPI_BUS_MASTER
 *
 * @brief Child class of Bus_Master_Device class which implements
 * function calls specific for SPI bus.
 */
class SPI_Bus_Master_Device: public Bus_Master_Device
{
	uint32_t m_spi_bus_max_speed_hz = 0;					/**< Maximal SPI bus speed in Hz. Value dictated by the SPI slave. */
	SPI_BUS_MODE m_spi_bus_mode = SPI_BUS_MODE::MODE_0;		/**< SPI mode (polarity and phase of the bus signal). */
	uint16_t m_spi_delay = 0;								/**< how long to delay after the last bit transfer before optionally deselecting the device before the next transfer. As described in the description of the spi_ioc_transfer structure. */
	uint8_t m_spi_bits_per_word = 8;						/**< Device's word size. Defined based on slave devices requirements. */

public:
	SPI_Bus_Master_Device(): Bus_Master_Device("/dev/spidev0.1", BUS_TYPE::SPI_BUS) {};
	/**
	 * @brief Class constructor. Provides the values of the SPI bus properties and the data required
	 * for parent class initialization.
	 *
	 * @param bus_master_device_file_name
	 */
	explicit SPI_Bus_Master_Device(const std::string &bus_master_device_file_name):
		Bus_Master_Device(bus_master_device_file_name, BUS_TYPE::SPI_BUS)
	{};

	/**
	 * @brief Class desctructor.
	 */
	~SPI_Bus_Master_Device() {};

	/**
	 * @brief Sets the parameters of the SPI bus. If not later overwritten by SPI slaves init/configure function,
	 * these parameters will be used by the SPI slave objects for data exchange on the bus.
	 *
	 * @param[in] spi_bus_mode  SPI mode (polarity and phase of the bus signal).
	 * @param[in] spi_bus_max_speed_hz	Maximal SPI bus speed in Hz. Value dictated by the SPI slave.
	 * @param[in] spi_bits_per_word Device's word size. Defined based on slave devices requirements.
	 * @param[in] spi_delay	how long to delay after the last bit transfer before optionally deselecting the device before the next transfer.
	 * @return
	 */
	int configure(SPI_BUS_MODE spi_bus_mode, uint32_t spi_bus_max_speed_hz, uint8_t spi_bits_per_word, uint16_t spi_delay);

	/**
	 * @brief Returns the bus speed property in Hz.
	 * @return value of the m_spi_bus_max_speed_hz member.
	 */
	uint32_t bus_max_speed_hz() const { return m_spi_bus_max_speed_hz; };

	/**
	 * @brief Returns the bus mode property.
	 * @return value of the m_spi_bus_mode
	 */
	SPI_BUS_MODE bus_mode() const { return m_spi_bus_mode; };

	/**
	 * @brief Returns the word size property.
	 *
	 * @return value of the m_spi_bits_per_word member.
	 */
	uint8_t bits_per_word() const { return m_spi_bits_per_word; };

	/**
	 * @brief Returns the delay property.
	 *
	 * @return value of the m_spi_delay member
	 */
	uint16_t delay() const { return m_spi_delay; };
};

#endif /* SPI_BUS_MASTER_H_ */
