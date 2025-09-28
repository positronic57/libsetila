/**
 * @file setila_version.h
 *
 * @date Sep 21, 2025
 *
 * @brief Macros and functions related to libsetila version information.
 *
 * The implementation is a copy from "MongoDB C Drive" code
 * (https://mongoc.org), pulishd under Apache License v2.
 *
 * @copyright GNU General Public License v3
 */

#ifndef SETILA_VERSION_H
#define SETILA_VERSION_H

#define SETILA_MAJOR_VERSION (0)
#define SETILA_MINOR_VERSION (5)
#define SETILA_PATCH_VERSION (9)

/**
 * @brief Libsetila version as a string.
 */
#define SETILA_VERSION_STRING "0.5.9"

/**
 *  @brief Libsetila version encoded as a hexadecimal number.
 */
#define SETILA_VERSION_HEX                                                     \
  (SETILA_MAJOR_VERSION << 16 | SETILA_MINOR_VERSION << 8 |                    \
   SETILA_PATCH_VERSION)

/**
 * @brief Compile-time version checking.
 * Evaluates to TRUE if the version of libsetila is greater
 * or equal to the required one.
 */
#define SETILA_CHECK_VERSION(major, minor, patch)                              \
  (SETILA_MAJOR_VERSION > (major) ||                                           \
   (SETILA_MAJOR_VERSION == (major) && SETILA_MINOR_VERSION > (minor)) ||      \
   (SETILA_MAJOR_VERSION == (major) && SETILA_MINOR_VERSION == (minor) &&      \
    SETILA_PATCH_VERSION >= (patch)))

/**
 * @brief Helper function to return the runtime major version of the library.
 */
int setila_get_major_version(void);

/**
 * @brief Helper function to return the runtime micro version of the library.
 */
int setila_get_minor_version(void);

/**
 * @brief Helper function to return the runtime patch version of the library.
 */
int setila_get_patch_version(void);

/**
 * @brief Helper function to return the runtime string version of the library.
 */
const char *setila_get_version_string(void);

/**
 * @brief  True if the version of libsetila is greater than or equal to the
 * required version.
 */
bool setila_check_version(int required_major, int required_minor,
                          int required_patch);

/**
 * @brief Helper function to return the runtime version of the library as
 * encoded as a hexadecimal number.
 */
unsigned long setila_get_version_hex(void);

#endif // SETILA_VERSION_H
