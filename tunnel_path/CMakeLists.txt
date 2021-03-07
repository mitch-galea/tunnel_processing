cmake_minimum_required(VERSION 3.0.2)
project(tunnel_path)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  roslib
  tf
  tf2
  tf2_ros
  ocgrid
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# generate_dynamic_reconfigure_options(
#         cfg/tunnel_path_settings.cfg
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
  src/${PROJECT_NAME}/tunnel_path.cpp
)
## Library Dependencies
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Link Library
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


## Node Executable
add_executable(${PROJECT_NAME}-node src/tunnel_path_node.cpp)

## Node Dependencies
add_dependencies(${PROJECT_NAME}-node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Node Linking
target_link_libraries(${PROJECT_NAME}-node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Testing ##
#############

catkin_add_gtest(${PROJECT_NAME}-test test/tunnel_path_test.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}-test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


