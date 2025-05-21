cmake_minimum_required(VERSION 2.8.3)
project(tug_cfg)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  roslint
  rospy
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS
  system
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

###########
## Build ##
###########

## Specify additional compile options for this package
add_definitions(-std=c++11 -Wformat -Wall -Wextra -Werror)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/collection.cpp include/${PROJECT_NAME}/collection.h
  src/configuration.cpp include/${PROJECT_NAME}/configuration.h
  src/default_constrainer.cpp include/${PROJECT_NAME}/default_constrainer.h
  src/error_handler.cpp include/${PROJECT_NAME}/error_handler.h
  src/log_error_handler.cpp include/${PROJECT_NAME}/log_error_handler.h
  src/ros_dynamic_config_describer.cpp include/${PROJECT_NAME}/ros_dynamic_config_describer.h
  src/ros_dynamic_config_reader.cpp include/${PROJECT_NAME}/ros_dynamic_config_reader.h
  src/ros_dynamic_config_server.cpp include/${PROJECT_NAME}/ros_dynamic_config_server.h
  src/ros_dynamic_config_writer.cpp include/${PROJECT_NAME}/ros_dynamic_config_writer.h
  include/${PROJECT_NAME}/forwards.h
  src/key.cpp include/${PROJECT_NAME}/key.h
  include/${PROJECT_NAME}/object.h
  src/ros_param_reader.cpp include/${PROJECT_NAME}/ros_param_reader.h
  include/${PROJECT_NAME}/scalar.h
  src/struct.cpp include/${PROJECT_NAME}/struct.h
  include/${PROJECT_NAME}/type.h
  include/${PROJECT_NAME}/visitor.h
  src/yaml_reader.cpp include/${PROJECT_NAME}/yaml_reader.h
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/tug_cfg_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

#add_custom_command(
#  COMMENT "Copying scripts to devel"
#  OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/generate_cpp.py
#  DEPENDS ${PROJECT_SOURCE_DIR}/scripts/generate_cpp.py
#  COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
#  COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/scripts/generate_cpp.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/generate_cpp.py
#)
#
#add_custom_target(${PROJECT_NAME}_copy_scripts ALL
#  DEPENDS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/generate_cpp.py
#)

# Target simply for adding the extras file to the list of source files (for IDEs):
add_custom_target(${PROJECT_NAME}_cmake_extras ALL
  SOURCES cmake/${PROJECT_NAME}-extras.cmake.em
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

catkin_install_python(
  PROGRAMS scripts/generate_cpp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
roslint_cpp()
roslint_add_test()

