# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(demoAvoidance)

set(sources demoAvoidance.cpp)
source_group("Source Files" FILES ${sources})

add_executable(demoAvoidance ${sources})
target_link_libraries(demoAvoidance periPersonalSpace ${YARP_LIBRARIES} ctrlLib)
install(TARGETS demoAvoidance DESTINATION bin)

