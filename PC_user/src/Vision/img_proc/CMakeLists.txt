cmake_minimum_required(VERSION 3.0.2)
project(img_proc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  ros_numpy
  sensor_msgs
  image_transport
  cv_bridge
  tf2_ros
)

## Find OpenCV
find_package(OpenCV REQUIRED)

## Find Eigen3
find_package(Eigen3 REQUIRED)

## Find ZBar using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)

## Declare ROS messages, services and actions ##
add_message_files(
  FILES
  MPS_Detector.msg
)

add_service_files(
  FILES
  Tag_with_tf.srv
  ReadQRCode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare catkin specific configuration ##
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${ZBAR_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(aruco_tf src/aruco_tf.cpp)
add_executable(aruco_tf_srv src/aruco_tf_srv.cpp)
add_executable(qr_srv src/qr_srv.cpp)

## Link libraries to the executable
target_link_libraries(aruco_tf
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} # Link OpenCV libraries
)

target_link_libraries(aruco_tf_srv
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} # Link OpenCV libraries
)

target_link_libraries(qr_srv
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${ZBAR_LIBRARIES} # Link ZBar libraries
)

## Add dependencies for generated messages and services
add_dependencies(aruco_tf     ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(aruco_tf_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(qr_srv       ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# Install the executable
install(TARGETS aruco_tf
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
