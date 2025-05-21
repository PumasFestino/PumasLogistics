cmake_minimum_required(VERSION 3.0.2)
project(pose_estimation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  visualization_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  Keypoint2D.msg
  PersonPose2D.msg
  Keypoint3D.msg
  PersonPose3D.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  roscpp
  rospy
  visualization_msgs
  sensor_msgs
)

# Python nodes
catkin_install_python(PROGRAMS
  scripts/yolo_pose_2d_node.py
  #scripts/markers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# C++ nodes
add_executable(pose_2d_to_3d_node src/pose_2d_to_3d_node.cpp)
add_dependencies(pose_2d_to_3d_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(pose_2d_to_3d_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(pointing_detector_node src/pointing_detector_node.cpp)
add_dependencies(pointing_detector_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(pointing_detector_node ${catkin_LIBRARIES})

add_executable(markers src/markers.cpp)
add_dependencies(markers ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(markers ${catkin_LIBRARIES})