cmake_minimum_required(VERSION 3.5)
project(crowd_simulation_gazebo)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(crowd_simulation_common REQUIRED)

# TODO this is a dependency of building_sim_common, it shouldn't be needed
find_package(Eigen3 REQUIRED)

find_package(menge REQUIRED)

include(GNUInstallDirs)

###############################
# crowd simulator stuff
###############################
add_library(crowd_simulator
  SHARED
    src/crowd_simulator.cpp
)

target_include_directories(crowd_simulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GAZEBO_INCLUDE_DIRS}
    ${menge_INCLUDE_DIRS}
    ${crowd_simulation_common_INCLUDE_DIRS}   
)

target_link_libraries(crowd_simulator
  PUBLIC
    ${GAZEBO_LIBRARIES}
    ${crowd_simulation_common_LIBRARIES}
)

###############################
# install stuff
###############################
ament_export_include_directories(include)

install(
  # TARGETS slotcar door toggle_floors
  TARGETS crowd_simulator
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/test/crowd_simulation/models
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/menge_resource
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
