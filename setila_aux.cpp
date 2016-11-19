 /**
  * @file setila_aux.cpp
  * @brief Implementation of auxiliary functions defined in setila_aux.h.
  *
  * @author Goce Boshkovski
  * @date 12-Nov-16
  * @copyright GNU General Public License v2.
  *
  */

#include "setila_aux.h"
#include "math.h"

int altitude(double lastPressureReading,double baseLinePressure)
{
	return round(44330*(1-pow((lastPressureReading/baseLinePressure),(1/5.255))));
}

