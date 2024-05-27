cmake_minimum_required(VERSION 3.0.2)
project(xjh_learn)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
std_msgs
actionlib
actionlib_msgs 
geometry_msgs
sensor_msgs
message_generation
nav_msgs
roscpp
rospy
tf
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_message_files(
  DIRECTORY msg 
  FILES
  DMVel.msg
  DMDema.msg
  DMGoal.msg
  DMPose.msg
)

add_service_files(
  DIRECTORY srv 
  FILES
  DMDemarcate.srv
)
add_action_files(
  DIRECTORY action
  FILES 
  DMDemarcate.action
)
generate_messages(
  DEPENDENCIES 
  std_msgs
  actionlib_msgs
  )

catkin_package(CATKIN_DEPENDS 
geometry_msgs
message_runtime 
actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

add_executable(xjh_node src/xjh_node.cpp)
target_link_libraries(xjh_node ${catkin_LIBRARIES})
