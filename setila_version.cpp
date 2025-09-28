/**
 * @file setila_version.cpp
 *
 * @date Sep 21, 2025
 *
 * The implementation is a copy from "MongoDB C Drive" code
 * (https://mongoc.org), pulishd under Apache License v2.
 *
 * @copyright GNU General Public License v3
 */

#include "setila_version.h"

int setila_get_major_version(void) { return SETILA_MAJOR_VERSION; }

int setila_get_minor_version(void) { return SETILA_MINOR_VERSION; }

int setila_get_micro_version(void) { return SETILA_PATCH_VERSION; }

const char *setila_get_version(void) { return SETILA_VERSION_STRING; }

bool setila_check_version(int required_major, int required_minor,
                          int required_micro) {
  return SETILA_CHECK_VERSION(required_major, required_minor, required_micro);
}

unsigned long setila_get_version_hex(void) { return SETILA_VERSION_HEX; }
