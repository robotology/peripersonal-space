# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(visuoTactileRF)

file(GLOB source *.cpp)
file(GLOB header *.h)

include_directories(${ICUB_INCLUDE_DIRS})

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(testSkinDynLib ${source} ${header} )

target_link_libraries(testSkinDynLib ${YARP_LIBRARIES} ${ICUB_LIBRARIES})

install(TARGETS testSkinDynLib DESTINATION bin)

