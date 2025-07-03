cmake_minimum_required(VERSION 3.0.2)
project(robot_to_main_communication)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
  InstructionService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_to_main_communication_node src/robot_to_main_communication_node.cpp)
add_dependencies(robot_to_main_communication_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_to_main_communication_node ${catkin_LIBRARIES})
