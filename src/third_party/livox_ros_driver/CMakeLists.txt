# Copyright(c) 2019 livoxtech limited.

cmake_minimum_required(VERSION 3.0)


#---------------------------------------------------------------------------------------
# Start livox_ros_driver project
#---------------------------------------------------------------------------------------
include(cmake/version.cmake)
project(livox_ros_driver VERSION ${LIVOX_ROS_DRIVER_VERSION} LANGUAGES CXX)
message(STATUS "livox_ros_driver version: ${LIVOX_ROS_DRIVER_VERSION}")

#---------------------------------------------------------------------------------------
# find package and the dependecy
#---------------------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    )

## Generate messages in the 'msg' folder
add_message_files(FILES
    CustomPoint.msg
    CustomMsg.msg
#   Message2.msg
    )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
    std_msgs
    )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS
    roscpp rospy std_msgs message_runtime
    )

# No executable compilation

#---------------------------------------------------------------------------------------
# end of CMakeList.txt
#---------------------------------------------------------------------------------------