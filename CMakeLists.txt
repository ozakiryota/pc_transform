cmake_minimum_required(VERSION 2.8.3)
project(pc_transform)

add_compile_options(-std=c++14 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)
find_package(PCL)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pc_tf_transform src/pc_tf_transform.cpp)
target_link_libraries(pc_tf_transform
  ${catkin_LIBRARIES}
)

add_executable(pc_transform src/pc_transform.cpp)
target_link_libraries(pc_transform
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
