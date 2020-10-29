cmake_minimum_required(VERSION 3.0.2)
project(pkg_task1)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)
add_action_files(
  DIRECTORY action
  FILES msgTurtle.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

