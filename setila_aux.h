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

/** \defgroup CONST Average sea level pressure. */
/* @{ */
#define AVERAGE_SEA_LEVEL_PRESSURE 1013.25
/* @} */

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
int altitude(double lastPressureReading,double baseLinePressure);

#endif /* SETILA_AUX_H_ */
