cmake_minimum_required(VERSION 3.5)
project(crowd_simulation_ign)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(menge REQUIRED)

find_package(ignition-cmake2 REQUIRED)
ign_find_package(ignition-gazebo4 REQUIRED)
set(IGN_GAZEBO_VER 4)
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER 1)
ign_find_package(ignition-common3 REQUIRED)
set(IGN_COMMON_VER 3)
ign_find_package(ignition-math4 REQUIRED)
set(IGN_MATH_VER 4)
ign_find_package(sdformat9 REQUIRED)

ign_find_package(ignition-transport8 REQUIRED)
set(IGN_TRANSPORT_VER 8)
ign_find_package(ignition-msgs5 REQUIRED)
set(IGN_MSGS_VER 5)


find_package(crowd_simulation_common REQUIRED)

include(GNUInstallDirs)

###############################
# Crowd Simulator stuff
###############################

add_library(crowd_simulator_ign
  SHARED
  src/crowd_simulator.cpp
)

target_include_directories(crowd_simulator_ign
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${crowd_simulation_common_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${menge_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
)

target_link_libraries(crowd_simulator_ign
  PUBLIC
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ${rclcpp_LIBRARIES}
  ${crowd_simulation_common_LIBRARIES}
  ${menge_LIBRARIES}
)

###############################
# install stuff
###############################

install(
  TARGETS crowd_simulator_ign
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/test/crowd_simulation/models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
