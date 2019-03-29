cmake_minimum_required(VERSION 2.8.3)
project(autonomous_husky)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
