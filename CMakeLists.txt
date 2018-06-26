# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(PPS)

find_package(YARP REQUIRED COMPONENTS OS dev sig math gsl)
find_package(ICUB)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

## add libraries
add_subdirectory(lib)

## then modules
add_subdirectory(modules)

## then apps
add_subdirectory(app)

icubcontrib_finalize_export(periPersonalSpace)
icubcontrib_add_uninstall_target()

