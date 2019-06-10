cmake_minimum_required(VERSION 2.8.3)
project(autonomous_husky)
project(barc.workspace.src.barc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
)

add_message_files(
  FILES
  barc.msg
)


catkin_package()
