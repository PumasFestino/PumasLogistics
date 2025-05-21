cmake_minimum_required(VERSION 3.0.2)
project(vosk_speech_recognition)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  speech_recog.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(

  CATKIN_DEPENDS rospy std_msgs message_runtime  # Dependencias de ROS
)