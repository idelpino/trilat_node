cmake_minimum_required(VERSION 2.8.3)
project(trilateration)

## to activate c++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  #std_msgs #TODO serve?
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Ceres REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   satMeasurement.msg
   satMeasurementArray.msg
   receiver.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime
)


###############
## Srcs list ##
###############

##SRCS files. Trilateration library
SET(SRCS_TR_LIB
	include/trilateration/src/trilateration.cpp
)

##SRCS files. Rinex reader library
SET(SRCS_RINEX_READER_LIB
	include/rinex_reader/src/rinex_reader.cpp
)

##SRCS files. ROS receiver simulator node
SET(SRCS_ROS_REC_SIM_NODE
	src/receiver_sim_node.cpp
	src/receiver_sim_main.cpp
)

##SRCS files. ROS trilaterate node
SET(SRCS_ROS_TRILATERATION_NODE
	src/trilateration_node.cpp
	src/trilateration_main.cpp
)

##SRCS files. ROS rinex reader node
SET(SRCS_ROS_RINEX_READ_NODE
	src/rinex_reader_node.cpp
	src/rinex_reader_main.cpp
)

##SRCS files. ROS orbit prediction node
SET(SRCS_ROS_ORBIT_PREDICT_NODE
	src/orbit_prediction_node.cpp
	src/orbit_prediction_main.cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## cpp executable
add_executable(receiver_sim_node ${SRCS_ROS_REC_SIM_NODE} ${SRCS_TR_LIB})
target_link_libraries(receiver_sim_node ${catkin_LIBRARIES} ${CERES_LIBRARIES})


add_executable(trilateration_node ${SRCS_ROS_TRILATERATION_NODE} ${SRCS_TR_LIB})
target_link_libraries(trilateration_node ${catkin_LIBRARIES} ${CERES_LIBRARIES})


add_executable(rinex_reader_node ${SRCS_ROS_RINEX_READ_NODE} ${SRCS_TR_LIB})
target_link_libraries(rinex_reader_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} gpstk)


add_executable(orbit_prediction_node ${SRCS_ROS_ORBIT_PREDICT_NODE} ${SRCS_RINEX_READER_LIB})
target_link_libraries(orbit_prediction_node ${catkin_LIBRARIES} gpstk)

