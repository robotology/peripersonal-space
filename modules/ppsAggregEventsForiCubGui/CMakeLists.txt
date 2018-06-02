# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Matej Hoffmann
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(ppsAggregEventsForiCubGui)

file(GLOB source *.cpp)
file(GLOB header *.h)

include_directories(${periPersonalSpace_INCLUDE_DIRS})

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(ppsAggregEventsForiCubGui ${source} ${header} )

target_link_libraries(ppsAggregEventsForiCubGui periPersonalSpace ${YARP_LIBRARIES})

install(TARGETS ppsAggregEventsForiCubGui DESTINATION bin)

