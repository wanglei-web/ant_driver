cmake_minimum_required(VERSION 2.8.3)
project(path_planning)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  driverless_actions
  actionlib
  actionlib_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/global_path_planning.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
# add dependencies to firstly build what ${PROJECT_NAME}_node need
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

