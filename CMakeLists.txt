cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Define the project name
project(libsetila)

include(GNUInstallDirs)

# To see the compiler commands change the value to ON
set(CMAKE_VERBOSE_MAKEFILE ON)

# Set the C++ compiler flags. Enable C++11 standard.
set(CMAKE_CXX_FLAGS "-std=c++0x -O0 -Wall -fmessage-length=0")

# Define the include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add the source files from "i2c" folder to the build.
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/i2c I2C_SOURCES)

# Add the source files from "spi" folder to the build.
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/spi SPI_SOURCES)

# Add the source files from "devices" folder to the build.
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/devices DEVICES_SOURCES)

# Define the list of source files.
set (SOURCE_LIST
${I2C_SOURCES}
${SPI_SOURCES}
${DEVICES_SOURCES}
bus_master_device.cpp
slave_device.cpp
)

# Build static and shared versions of the library.
add_library(setila SHARED ${SOURCE_LIST})
add_library(setila_stat STATIC ${SOURCE_LIST})

# Set the name of the setila_stat target
set_target_properties(setila_stat PROPERTIES OUTPUT_NAME setila)

# Create target directories
install(DIRECTORY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/lib)

install (TARGETS setila setila_stat
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/setila
        FILES_MATCHING PATTERN "*.h")
