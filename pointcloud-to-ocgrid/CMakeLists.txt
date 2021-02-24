cmake_minimum_required(VERSION 3.0.2)
project(pointcloud-to-ocgrid)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  roslib
  tf
  tf2
  tf2_ros
  ocgrid
)

find_package(PCL 1.7 REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Library 
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/pointcloud-to-ocgrid.cpp
)
## Library Dependencies
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Node Executable
add_executable(${PROJECT_NAME}-node src/pointcloud-to-ocgrid-node.cpp)

## Node Dependencies
add_dependencies(${PROJECT_NAME}-node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Node Linking
target_link_libraries(${PROJECT_NAME}-node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Testing ##
#############

catkin_add_gtest(${PROJECT_NAME}-test test/pointcloud-to-ocgrid-test.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})


