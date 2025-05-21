@[if DEVELSPACE]@
# bin and template dir variables in develspace:
set(TUG_CFG_SCRIPTS_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
@[else]@
# bin and template dir variables in installspace:
set(TUG_CFG_SCRIPTS_DIR "${tug_cfg_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)")
@[end if]@

# This calls tug_cfg scripts to generate header files for given configuration
# definitions:
macro(tug_cfg_generate_cpp)
  _tug_cfg_init()

  include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})

  # Add actual workspace to include directories (for Qt Creator):
  list(FIND CATKIN_WORKSPACES "${CATKIN_DEVEL_PREFIX}" _index)
  if(_index EQUAL -1)
    list(GET CATKIN_WORKSPACES 0 _ws)
    include_directories(${_ws}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
  endif()

  foreach(_arg ${ARGN})
    set(_cfg_file_path ${PROJECT_SOURCE_DIR}/${_arg})
    get_filename_component(_cfg_name ${_cfg_file_path} NAME_WE)
    set(_header_file_path ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${_cfg_name}.h)

    add_custom_command(
      COMMENT "Generating C++ code from ${PROJECT_NAME}/${_arg}"
      OUTPUT ${_header_file_path}
      DEPENDS ${_cfg_file_path} ${TUG_CFG_SCRIPTS_DIR}/generate_cpp.py
      COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      # CATKIN_ENV prepares some environment variables:
      COMMAND ${CATKIN_ENV}
              env "PYTHONPATH=${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}:$$PYTHONPATH"
              ${TUG_CFG_SCRIPTS_DIR}/generate_cpp.py
              ${PROJECT_NAME} ${_cfg_file_path} > ${_header_file_path}
    )

    add_custom_target(${PROJECT_NAME}_tug_cfg_generate_cpp_${_cfg_name}
      DEPENDS ${_header_file_path}
      SOURCES ${_arg}
    )

    add_dependencies(${PROJECT_NAME}_tug_cfg ${PROJECT_NAME}_tug_cfg_generate_cpp_${_cfg_name})

    install(FILES ${_header_file_path} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  endforeach()
endmacro()


macro(_tug_cfg_init)
  if (NOT TARGET ${PROJECT_NAME}_tug_cfg)
    if(${PROJECT_NAME}_CATKIN_PACKAGE)
      message(FATAL_ERROR "${PROJECT_NAME}: tug_cfg_... macro called after catkin_package()")
    endif()

    # Make sure package destination variables are defined:
    catkin_destinations()

    add_custom_target(${PROJECT_NAME}_tug_cfg ALL)
    list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_tug_cfg)
  endif()
endmacro()
