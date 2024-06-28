# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_imu_heading_pkg_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED imu_heading_pkg_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(imu_heading_pkg_FOUND FALSE)
  elseif(NOT imu_heading_pkg_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(imu_heading_pkg_FOUND FALSE)
  endif()
  return()
endif()
set(_imu_heading_pkg_CONFIG_INCLUDED TRUE)

# output package information
if(NOT imu_heading_pkg_FIND_QUIETLY)
  message(STATUS "Found imu_heading_pkg: 0.0.0 (${imu_heading_pkg_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'imu_heading_pkg' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${imu_heading_pkg_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(imu_heading_pkg_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${imu_heading_pkg_DIR}/${_extra}")
endforeach()
