#ifndef SETILA_AUX_H
#define SETILA_AUX_H

#include <cstdint>

namespace aux {

/**
 * @ibrief Creates a new signed word from two unsigned byte values.
 *
 * @param[in] MSByte a byte value that will become the most significant byte of
 * the word
 * @param[in] LSByte a byte value that will become the least significant
 * byte of the word
 *
 * @return a signed 16-bit word
 */
int16_t createSignedWord(uint8_t MSByte, uint8_t LSByte);

/**
 * @ibrief Creates a new signed word from two unsigned byte values.
 *
 * @param[in] MSByte a byte value that will become the most significant byte of
 * the word
 * @param[in] LSByte a byte value that will become the least significant
 * byte of the word
 *
 * @return an unsigned 16-bit word
 */
uint16_t createUnsignedWord(uint8_t MSByte, uint8_t LSByte);

/**
 * @brief Calculates the relative altitude changes in [m] based on pressure
 * difference between two measurement points.
 *
 * If the average sea level pressure is used as a reference point,
 * the function will return the altitude above sea level in [m].
 *
 * @param[in] lastPressureReading Last pressure reading.
 * @param[in] baseLinePressure Reference pressure for calculating the relative
 * altitude changes.
 *
 * @return int The relative altitude change due to difference in pressure
 * values.
 */
int atmosphericPressure2Altitude(float atmosphericPressure,
                                 float baseLinePressure);

} // namespace aux

#endif
