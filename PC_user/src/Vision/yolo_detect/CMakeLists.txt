cmake_minimum_required(VERSION 3.0.2)
project(yolo_detect)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
)

add_message_files(
  FILES
  StringArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/yolo_detect_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
