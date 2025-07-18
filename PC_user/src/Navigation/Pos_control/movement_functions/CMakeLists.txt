cmake_minimum_required(VERSION 3.0.2)
project(movement_functions)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  nav_msgs
  message_generation
  roslib
)

add_service_files(
  FILES
  ModifyMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs nav_msgs message_runtime roslib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(zones_node src/zones_node.cpp)
target_link_libraries(zones_node ${catkin_LIBRARIES})

add_executable(reload_amcl_service src/reload_amcl_service.cpp)
add_dependencies(reload_amcl_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reload_amcl_service ${catkin_LIBRARIES})


add_executable(mod_map_node src/mod_map_node.cpp)
target_link_libraries(mod_map_node ${catkin_LIBRARIES})
add_dependencies(mod_map_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
