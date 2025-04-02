# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_teamb_global_path_planner_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED teamb_global_path_planner_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(teamb_global_path_planner_FOUND FALSE)
  elseif(NOT teamb_global_path_planner_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(teamb_global_path_planner_FOUND FALSE)
  endif()
  return()
endif()
set(_teamb_global_path_planner_CONFIG_INCLUDED TRUE)

# output package information
if(NOT teamb_global_path_planner_FIND_QUIETLY)
  message(STATUS "Found teamb_global_path_planner: 0.0.0 (${teamb_global_path_planner_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'teamb_global_path_planner' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${teamb_global_path_planner_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(teamb_global_path_planner_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${teamb_global_path_planner_DIR}/${_extra}")
endforeach()
