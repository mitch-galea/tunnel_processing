cmake_minimum_required(VERSION 2.8.3)
project(ocgrid)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  map_server
  roscpp
  std_msgs
  tf2
  tf2_ros
  roslib
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)



###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ocgrid
#  CATKIN_DEPENDS nav_msgs roscpp std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}/ocgrid.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Library Dependencies
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/ocgrid-test.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
