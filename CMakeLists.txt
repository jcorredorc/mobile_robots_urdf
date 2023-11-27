cmake_minimum_required(VERSION 3.0.2)
project(my_ugv_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf #Required for tf/transform_broadcaster
)

catkin_package( CATKIN_DEPENDS roscpp rospy std_msgs tf)

include_directories( ${catkin_INCLUDE_DIRS})

add_executable(omni_tf_4wheels src/omni_tf_4wheels.cpp)
target_link_libraries(omni_tf_4wheels ${catkin_LIBRARIES})

