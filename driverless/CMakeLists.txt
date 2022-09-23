cmake_minimum_required(VERSION 2.8.3)
project(driverless)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  driverless_actions
  actionlib
  actionlib_msgs
  perception_msgs
  obu_msgs
  light_msgs
  diagnostic_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  driverless_common
)

find_package(OpenCV REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)



add_executable(${PROJECT_NAME}_node src/auto_drive_base.cpp 
                                    src/driverless_node.cpp 
                                    src/driverless_fun.cpp
                                    src/local_path_planning.cpp
                                    src/path_tracking.cpp)

target_include_directories(${PROJECT_NAME}_node PRIVATE ${OpenCV_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} /usr/local/lib/libtinyxml2.a)

# add dependencies to firstly build what ${PROJECT_NAME}_node need
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

