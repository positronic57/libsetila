/**
 * @file setila.h
 *
 * @brief Includes all of header files from libsetila project.
 *
 * @author Goce Boshkovski
 * @date 21 Jul 2019
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef SETILA_H_
#define SETILA_H_

/* Include all the header files providing I2C support. */
#include "setila_i2c.h"

/* Include all the header files providing SPI support. */
#include "setila_spi.h"


/* Slave device specific definitions. */
#include "HTS221.h"
#include "LPS25H.h"
#include "srf02.h"
#include "BMP085.h"
#include "setila_errors.h"

/**
 * \defgroup SPI_BUS SPI Bus
 */

/**
 * \defgroup I2C_BUS I2C Bus
 */

/**
 * \defgroup SPI_BUS_MASTER SPI Bus Master
 * @ingroup SPI_BUS
 */

/**
 * \defgroup I2C_BUS_MASTER I2C Bus Master
 * @ingroup I2C_BUS
 */

/**
 * \defgroup DEV_REG_CMD Device Register Descriptors and Commands
 */
#endif /* SETILA_H_ */
