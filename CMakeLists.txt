cmake_minimum_required(VERSION 2.8.3)
project(autonomous_husky)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  OpenCV
  cv_bridge
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(my_awesome_library ${OpenCV_LIBRARIES})
