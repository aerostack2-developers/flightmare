# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_flightros_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED flightros_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(flightros_FOUND FALSE)
  elseif(NOT flightros_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(flightros_FOUND FALSE)
  endif()
  return()
endif()
set(_flightros_CONFIG_INCLUDED TRUE)

# output package information
if(NOT flightros_FIND_QUIETLY)
  message(STATUS "Found flightros: 0.0.1 (${flightros_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'flightros' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${flightros_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(flightros_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${flightros_DIR}/${_extra}")
endforeach()
