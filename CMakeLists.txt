# Copyright: Universidad Carlos III de Madrid (C) 2016
# Authors: Juan Miguel Garcia Haro
# CopyPolicy: Released under the terms of the GNU GPL v2.0

cmake_minimum_required(VERSION 3.16)

project(WBPC_WAITER LANGUAGES CXX)

# Let the user specify a configuration (only single-config generators).
if(NOT CMAKE_CONFIGURATION_TYPES)
    # Possible values.
    set(_configurations Debug Release MinSizeRel RelWithDebInfo)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${_configurations})

    foreach(_conf ${_configurations})
        set(_conf_string "${_conf_string} ${_conf}")
    endforeach()

    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY HELPSTRING
                 "Choose the type of build, options are:${_conf_string}")

    if(NOT CMAKE_BUILD_TYPE)
        # Encourage the user to specify build type.
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE Release)
    endif()
endif()

# Pick up our cmake modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
                              ${CMAKE_SOURCE_DIR}/cmake/find-modules)

# Hard dependencies.
find_package(YARP 3.8 REQUIRED COMPONENTS os dev sig)
find_package(COLOR_DEBUG REQUIRED)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)

# Configure installation paths for YARP resources.
yarp_configure_external_installation(roboticslab-kinematics-dynamics)

# Standard installation directories.
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

set(WBPC_WAITER_INCLUDE_DIRS CACHE INTERNAL "appended header dirs" FORCE)
set(WBPC_WAITER_LINK_DIRS CACHE INTERNAL "appended link dirs" FORCE)
set(WBPC_WAITER_LIBRARIES CACHE INTERNAL "appended libraries" FORCE)

# Add main contents.
add_subdirectory(share)
# add_subdirectory(libraries)
add_subdirectory(programs)

# Configure and create uninstall target.
include(AddUninstallTarget)
