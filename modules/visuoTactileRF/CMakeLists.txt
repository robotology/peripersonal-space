# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(visuoTactileRF)

file(GLOB source *.cpp)
file(GLOB header *.h)

include_directories(${periPersonalSpace_INCLUDE_DIRS})

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(visuoTactileRF ${source} ${header})

target_link_libraries(visuoTactileRF periPersonalSpace ${YARP_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS visuoTactileRF DESTINATION bin)

