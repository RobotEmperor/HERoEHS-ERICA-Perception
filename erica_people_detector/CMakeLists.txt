################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(erica_people_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  darknet_ros_msgs
  erica_perception_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(ZED 2.3)
find_package(CUDA)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
#################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    darknet_ros_msgs
    erica_perception_msgs
  DEPENDS Boost #EIGEN3
)


################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
#  ${EIGEN3_INCLUDE_DIRS}
#  ${YAML_CPP_INCLUDE_DIRS}
)

if(NOT DEFINED CUDA_NPP_LIBRARIES_ZED)
    #To deal with cuda 9 nppi libs and previous versions of ZED SDK
    set(CUDA_NPP_LIBRARIES_ZED ${CUDA_npp_LIBRARY} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY})   
endif()


add_definitions(-std=c++11)
set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES} ${CUDA_NPP_LIBRARIES_ZED}
  )

add_executable(${PROJECT_NAME} src/erica_people_detector.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${LINK_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})

