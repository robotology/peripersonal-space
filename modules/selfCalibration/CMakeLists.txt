# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(iCubSelfCalib)


find_package(IPOPT REQUIRED)
include_directories(${IPOPT_INCLUDE_DIRS})
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${periPersonalSpace_INCLUDE_DIRS})

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})


add_executable(iCubSelfCalib ${source} ${header} )

target_link_libraries(iCubSelfCalib periPersonalSpace ${YARP_LIBRARIES} ${IPOPT_LIBRARIES})

install(TARGETS iCubSelfCalib DESTINATION bin)
