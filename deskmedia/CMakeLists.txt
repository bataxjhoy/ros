cmake_minimum_required(VERSION 2.8.3)
project(deskmedia)

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
  include 
  ${catkin_INCLUDE_DIRS}
  /usr/include/
)

link_directories(/usr/lib/)

add_message_files(
  DIRECTORY msg 
  FILES
  DMVel.msg
  DMErr.msg
  DMMap.msg
  DMPose.msg
  DMDema.msg
  DMGoal.msg
  DMSetMap.msg
  DMSetPoint.msg
  DMSrcVel.msg
)

add_service_files(
  DIRECTORY srv 
  FILES
  DMCmd.srv
  DMDemarcate.srv
)

add_action_files(
  DIRECTORY action 
  FILES 
  DMCtrl.action
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
)

#catkin_install_python(PROGRAMS scripts/listener.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#add_executable(gocircle src/gocircle.cpp)
#target_link_libraries(gocircle ${catkin_LIBRARIES})

add_executable(common_node src/deskmedia_node.cpp)
target_link_libraries(common_node ${catkin_LIBRARIES} json)
