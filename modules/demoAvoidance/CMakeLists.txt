# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(demoAvoidance)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})

set(sources demoAvoidance.cpp)
source_group("Source Files" FILES ${sources})

add_executable(demoAvoidance ${sources})
target_link_libraries(demoAvoidance ${YARP_LIBRARIES} ctrlLib icubmod)
install(TARGETS demoAvoidance DESTINATION bin)

