cmake_minimum_required(VERSION 2.8.3)
project(pc_transform)

add_compile_options(-std=c++17 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  rosbag
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

add_executable(tf_transform src/tf_transform.cpp)
target_link_libraries(tf_transform
  ${catkin_LIBRARIES}
)

add_executable(eular_transform src/eular_transform.cpp)
target_link_libraries(eular_transform
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(rpy_transform src/rpy_transform.cpp)
target_link_libraries(rpy_transform
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(tf_transform_offline src/tf_transform_offline.cpp)
target_link_libraries(tf_transform_offline
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
