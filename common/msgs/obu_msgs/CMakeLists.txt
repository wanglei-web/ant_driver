cmake_minimum_required(VERSION 3.0.2)
project(obu_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  OBU_fusion.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

