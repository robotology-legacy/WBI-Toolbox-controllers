# Copyright (C) 2013 CoDyCo
# Author: Jorhabib Eljaik
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.11)
project(WBI-Toolbox-controllers)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(YARP        REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)

set(WBI-TOOLBOX-CONTROLLERS_SHARE_DIR "${CMAKE_INSTALL_PREFIX}/share/WBI-Toolbox-controllers")
# Fake installation. Just installs the README.md file
install(FILE ${CMAKE_CURRENT_SOURCE_DIR}/README.md DESTINATION ${WBI-TOOLBOX-CONTROLLERS_SHARE_DIR})
