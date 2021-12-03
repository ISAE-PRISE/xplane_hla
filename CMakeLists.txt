#----------------------------------------------------------------------
#----------------------------------------------------------------------
#
# SMARTIES 
# Simulation Modules for Aircraft Real-Time Embedded Systems
#
# XPLANE HLA PLUGIN
# 
# Copyright (C) 2018-2022  ISAE-SUPAERO
#
# Author: Jean-Baptiste Chaudron
# email:  jean-baptiste.chaudron@isae-supaero.fr
#
#----------------------------------------------------------------------
#----------------------------------------------------------------------
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(SMARTIES_HLA_XPLANE_PLUGIN)

#----------------------------------------------------------------------
#PLUGIN CONFIGURATION
set(RTI_INCLUDE_DIR $ENV{CERTI_HOME}/include)
set(RTI1516e_INCLUDE_DIR $ENV{CERTI_HOME}/include/ieee1516-2010)


set(PLUGIN_XPLANE_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/src/xplane_hla_plugin/XplaneFederateHla1516ePlugin.cc
	${CMAKE_CURRENT_SOURCE_DIR}/src/xplane_hla_plugin/XplaneFederateHla1516e.cc
	${CMAKE_CURRENT_SOURCE_DIR}/src/xplane_hla_plugin/RemoteAircraftToDraw.cc
)


set(CLIENT_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/test_federate/testFederate.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/xplane_hla_plugin/XplaneFederateHla1516e.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/xplane_hla_plugin/RemoteAircraftToDraw.cc
)
set(LIBRARIES_PATHS
        $ENV{CERTI_HOME}/lib
#        $ENV{CERTI_HOME}/lib/federate_lib_plugin
)

set(LINK_LIB
	-lGL
	-lGLU
#		-lfederate64
	-lRTI1516ed
	-lFedTime1516ed
)


#Use CERTI specific CMake modules first 
IF ("$ENV{CERTI_HOME}" STRGREATER "")
    FILE(TO_CMAKE_PATH "$ENV{CERTI_HOME}" CERTI_HOME_TEMP)
    SET(CERTI_HOME ${CERTI_HOME_TEMP} CACHE PATH "The CERTI install directory")
    LIST(APPEND CMAKE_MODULE_PATH "${CERTI_HOME}/share/scripts")

# Or try to use CERTI possible installed at the same INSTALL_PREFIX
ELSE ("$ENV{CERTI_HOME}" STRGREATER "")
    LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/share/scripts")    
ENDIF ("$ENV{CERTI_HOME}" STRGREATER "")


#Call i.e. FIND RTI  
FIND_PACKAGE(RTI REQUIRED)
INCLUDE_DIRECTORIES(${RTI_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${RTI1516e_INCLUDE_DIR})
ADD_DEFINITIONS(${RTI_DEFINITIONS})
MESSAGE(STATUS "* RTI FOUND *")

####################
#Project includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/xplane_hla_plugin)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/xplane_sdk/CHeaders/XPLM)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/xplane_sdk/CHeaders/Widgets)
include_directories(${RTI_INCLUDE_DIR} ${RTI1516e_INCLUDE_DIR} ${FEDERATE_INCLUDE_DIR})

link_directories(${LIBRARIES_PATHS})

add_definitions(-DXPLM200 -DXPLM300 -DXPLM301 -DLIN=1)

SET(CMAKE_C_COMPILER gcc)
SET(CMAKE_CXX_COMPILER g++)


add_library(xpc64 SHARED ${PLUGIN_XPLANE_SOURCES} fom/isae_prise_hla1516e.xml)	#${RTI1516e_LIBRARIES} 
########TEMP CONFIGURATION##########################
target_link_libraries(xpc64 PRIVATE ${LINK_LIB})
####################################################
set_target_properties(xpc64 PROPERTIES PREFIX "" SUFFIX ".xpl")
set_target_properties(xpc64 PROPERTIES COMPILE_FLAGS "-m64 -fvisibility=hidden -fno-stack-protector -std=c++14 -DXPLANE_PLUGIN" LINK_FLAGS "-shared -rdynamic -nodefaultlibs -undefined_warning -m64")

add_library(xpc32 SHARED ${PLUGIN_XPLANE_SOURCES} fom/isae_prise_hla1516e.xml)
########TEMP CONFIGURATION##########################
if(WIN32)
target_link_libraries(xpc32 PRIVATE ${LINK_LIB})
endif()
####################################################
set_target_properties(xpc32 PROPERTIES PREFIX "" SUFFIX ".xpl")
set_target_properties(xpc32 PROPERTIES COMPILE_FLAGS "-m32 -fvisibility=hidden -fno-stack-protector -std=c++14 -DXPLANE_PLUGIN" LINK_FLAGS "-shared -rdynamic -nodefaultlibs -undefined_warning -m32")

#SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fvisibility=hidden -fno-stack-protector -std=c++14")

install(TARGETS xpc64 DESTINATION XplaneCertiHla/lin_x64)
#install(TARGETS xpc64 DESTINATION $ENV{PLUGIN_PATH}/xpc64/lin_x64)
install(TARGETS xpc32 DESTINATION XplaneCertiHla/)


add_executable(client_test ${CLIENT_TEST_SOURCES} fom/isae_prise_hla1516e.xml) #${RTI1516e_LIBRARIES} 
target_link_libraries(client_test ${LINK_LIB})
set_target_properties(client_test PROPERTIES COMPILE_FLAGS "-DCLIENT_TEST")
#set_target_properties(client_test PROPERTIES COMPILE_FLAGS "-m64 -fno-stack-protector" LINK_FLAGS "-rdynamic -nodefaultlibs -undefined_warning -m64")
install(TARGETS client_test DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)
