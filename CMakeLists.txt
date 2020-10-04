cmake_minimum_required(VERSION 2.8.3)
project(autonomous_husky)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
)

catkin_package()
