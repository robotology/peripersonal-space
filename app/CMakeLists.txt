# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

file(GLOB scripts  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB conf     ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB taxels   ${CMAKE_CURRENT_SOURCE_DIR}/conf/taxelsFiles/*.ini)
file(GLOB taxelsV1 ${CMAKE_CURRENT_SOURCE_DIR}/conf/taxelsFiles/forearmSkinV1/*.ini)
                 
### create a single target that installs all applications at once
yarp_install(FILES ${conf}     DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/periPersonalSpace)
yarp_install(FILES ${taxels}   DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/periPersonalSpace/taxelsFiles)
yarp_install(FILES ${taxelsV1} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/periPersonalSpace/taxelsFiles/forearmSkinV1)
yarp_install(FILES ${scripts}  DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
