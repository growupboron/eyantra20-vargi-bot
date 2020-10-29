cmake_minimum_required(VERSION 3.0.2)
project(pkg_ros_iot_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  msgMqttSub.msg
)

add_action_files(
  DIRECTORY action
  FILES msgRosIot.action
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
