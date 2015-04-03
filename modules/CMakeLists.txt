# Copyright: (C) 2013 iCub Facility
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(OpenCV REQUIRED)

add_subdirectory(doubleTouch)
add_subdirectory(visuoTactileRF)
add_subdirectory(visuoTactileWrapper)
add_subdirectory(demoAvoidance)
add_subdirectory(ultimateTracker)
#add_subdirectory(tactileServoing)
add_subdirectory(fingertipTracker)
add_subdirectory(virtualContactGeneration)
#add_subdirectory(selfCalibration)
#add_subdirectory(sceneFlowMod)
#add_subdirectory(alignEyes)
