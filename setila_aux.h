 /**
  * @file setila_aux.h
  * @brief A header file from libsetila library.
  * It contains the definitions of the auxiliary functions
  * for processing the sensor readings.
  *
  * @author Goce Boshkovski
  * @date 12-Nov-16
  * @copyright GNU General Public License v2.
  *
  */

#ifndef SETILA_AUX_H_
#define SETILA_AUX_H_

/** \defgroup AUX_DESC Auxiliary definitions */

/** \defgroup CONST Constants
 * @ingroup AUX_DESC
 */
/* @{ */
/** @brief The average pressure at the sea level. Required for calculating the altitude from the pressure measurement.*/
#define AVERAGE_SEA_LEVEL_PRESSURE 1013.25
/* @} */


/**
 * \class SetilaAUX
 *
 * @brief A class that provides auxiliary functions.
 * For example: calculate an altitude based on a pressure reading from a sensor.
 */
class SetilaAUX {
public:
	/**
	 * @brief Calculates the relative altitude changes in [m] based on pressure difference between two measurement points.
	 *
	 * If the average sea level pressure is used as a reference point,
	 * the function will return the altitude above sea level in [m].
	 *
	 * @param[in] lastPressureReading Last pressure reading.
	 * @param[in] baseLinePressure Reference pressure for calculating the relative altitude changes.
	 * @return int The relative altitude change due to difference in pressure values.
	 */
	static int altitude(double lastPressureReading, double baseLinePressure);
};

#endif /* SETILA_AUX_H_ */
