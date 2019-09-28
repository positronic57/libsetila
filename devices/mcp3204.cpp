/**
 * @file mcp3204.cpp
 *
 * @brief Contains the implementation of MCP3204 class.
 *
 * @author Goce Boshkovski
 * @date 9 Aug 2019
 *
 * @copyright GNU General Public License v3
 *
 */

extern "C" {
#include <sys/ioctl.h>
}

#include <iostream>
#include <cstdint>

#include "mcp3204.h"
#include "spi_bus.h"
#include "setila_errors.h"

int MCP3204::configure(SPI_BUS_MODE mcp3204_SPI_mode, float ref_voltage)
{
	if ((mcp3204_SPI_mode == SPI_BUS_MODE::MODE_1) || (mcp3204_SPI_mode == SPI_BUS_MODE::MODE_2))
	{
		return ERROR_UNSUPPORTED_SPI_MODE;	// SPI mode unsupported by MCP3204
	}

	/* Set the SPI bus parameters for MCP3204. */
	if (spi_bus_master())
	{
		if (spi_bus_master()->bus_master_fd() != -1)
		{
			if (spi_bus_master()->configure(mcp3204_SPI_mode, MCP3204_SPI_BUS_SPEED, MCP3204_SPI_BITS_PER_WORD, 0) < 0)
			{
				return ERROR_CONF_BUS_MASTER_FAILED;
			}
		}
		else 
		{
			return ERROR_NOT_ATTACHED_TO_A_BUS;
		}
	} else 
	{
		return ERROR_NOT_ATTACHED_TO_A_BUS;
	}
	
	m_reference_voltage=ref_voltage;

	return 0;
}

/*
 * Start the AD conversion process and read the digital value
 * of the analog signal from MCP3204.
 */
int MCP3204::convert(MCP3204_INPUT_CHANNEL input_channel, MCP3204_INPUT_CHANNEL_MODE input_channel_mode)
{
	int ret = 0;

	unsigned char tx[] = {0x00, 0x00, 0x00};
	unsigned char rx[] = {0x00, 0x00, 0x00};

	/* Check if the slave device is connected to the SPI bus master and the bus is already open. */
	if (spi_bus_master() == nullptr){
		return -1;
	} else if (spi_bus_master()->bus_master_fd() == -1){
		return -1;
	}

	/* set the start bit */
	tx[0] |= START_BIT;

	/* Define the channel input mode */
	switch(input_channel_mode)
	{
	case MCP3204_INPUT_CHANNEL_MODE::SINGLE_ENDED:
		tx[0] |= static_cast<uint8_t>(input_channel_mode);
		break;
	case MCP3204_INPUT_CHANNEL_MODE::DIFFERENTIAL:
		tx[0] &= static_cast<uint8_t>(input_channel_mode);
		break;
	default:
		tx[0] |= static_cast<uint8_t>(MCP3204_INPUT_CHANNEL_MODE::SINGLE_ENDED);
		break;
	}

	/* set the input channel/pair */
	switch(input_channel)
	{
		case MCP3204_INPUT_CHANNEL::CH0:
		case MCP3204_INPUT_CHANNEL::CH01:
			tx[1] |= MCP3204_CH_0;
			break;
		case MCP3204_INPUT_CHANNEL::CH1:
		case MCP3204_INPUT_CHANNEL::CH10:
			tx[1] |= MCP3204_CH_1;
			break;
		case MCP3204_INPUT_CHANNEL::CH2:
		case MCP3204_INPUT_CHANNEL::CH23:
			tx[1] |= MCP3204_CH_2;
			break;
		case MCP3204_INPUT_CHANNEL::CH3:
		case MCP3204_INPUT_CHANNEL::CH32:
			tx[1] |= MCP3204_CH_3;
			break;
	}
	
	ret = read_write(tx, rx, 3);
	if (ret == 0)
	{
		rx[1] &= 0x0F;
		m_digital_value = rx[1];
		m_digital_value <<= 8;
		m_digital_value |= rx[2];
	}

	return ret;
}
