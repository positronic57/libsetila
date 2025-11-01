#include <cmath>

#include "include/setila_aux.h"

int16_t aux::createSignedWord(uint8_t MSByte, uint8_t LSByte) {
  return ((static_cast<int16_t>(MSByte)) << 8) | LSByte;
}

uint16_t aux::createUnsignedWord(uint8_t MSByte, uint8_t LSByte) {
  return ((static_cast<uint16_t>(MSByte)) << 8) | LSByte;
}

int aux::atmosphericPressure2Altitude(float atmosphericPressure,
                                      float baseLinePressure) {
  return std::round(
      44330 *
      (1 - std::pow((atmosphericPressure / baseLinePressure), (1 / 5.255))));
}
