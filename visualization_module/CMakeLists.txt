cmake_minimum_required(VERSION 2.8.3)
project(visualization_module)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  path_msgs
  roscpp
  rospy
  sensor_msgs
  smartcar_msgs
  tf
  visualization_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/adjust_config.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vis_v1
#  CATKIN_DEPENDS geometry_msgs nav_msgs path_msgs roscpp rospy sensor_msgs smartcar_msgs tf visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(main
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(dynamic_visualization src/dynamic_visualization.cpp)
target_link_libraries(dynamic_visualization ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(dynamic_visualization
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

