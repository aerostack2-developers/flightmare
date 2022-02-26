cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME flightros)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

option(BUILD_MP "Build Motion Planning" OFF)

# Setup Default Build Type as Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

# Setup Architcture-specific Flags
if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
    message(STATUS "Using ARMv7 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM aarch64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else ()
    set(CMAKE_CXX_ARCH_FLAGS " -march=native")
endif ()

# Setup General C++ Flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=1048576")
# otherwise double free or corruption (out) error when running racing or motion_planning example
add_compile_options(-O3)

# Setup Release and Debug flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} -Wall -DNDEBUG -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -g")


set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  std_msgs
  as2_msgs
  std_srvs
  nav_msgs
  sensor_msgs
  Eigen3
  OpenCV
  flightlib
  image_transport
  cv_bridge)

#add_subdirectory(../flightlib/ flightlib/)#CHAPUZA WARNING!!! <-----------

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES
src/pilot/flight_pilot.cpp
src/pilot/circuit_pilot.cpp
src/camera/camera.cpp
)

#pilot
add_executable(flight_pilot_node src/pilot/flight_pilot_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(flight_pilot_node ${PROJECT_DEPENDENCIES})
target_link_libraries(flight_pilot_node ${OpenCV_LIBRARIES} ${flightlib_LIBRARIES} stdc++fs zmqpp)

add_executable(circuit_pilot_node src/pilot/circuit_pilot_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(circuit_pilot_node ${PROJECT_DEPENDENCIES})
target_link_libraries(circuit_pilot_node ${OpenCV_LIBRARIES} ${flightlib_LIBRARIES} stdc++fs zmqpp)

# camera
add_executable(camera_node src/camera/camera_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(camera_node ${PROJECT_DEPENDENCIES})
target_link_libraries(camera_node
${OpenCV_LIBRARIES}
${flightlib_LIBRARIES}
stdc++fs
zmq
zmqpp
)

# Finish
# cs_install()
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# cs_export()
install(TARGETS
        flight_pilot_node
        circuit_pilot_node
        camera_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()

