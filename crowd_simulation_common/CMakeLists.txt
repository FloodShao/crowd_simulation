cmake_minimum_required(VERSION 3.5)
project(crowd_simulation_common)

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
find_package(Eigen3 REQUIRED)
# find_package(sdformat9 REQUIRED)
find_package(gazebo REQUIRED)
# find_package(gazebo_ros REQUIRED)
# find_package(gazebo_dev REQUIRED)
# find_package(gazebo_msgs REQUIRED)

find_package(menge REQUIRED)
find_package(ignition-math6 REQUIRED)

include(GNUInstallDirs)

###############################
# crowd simulator stuff
###############################
add_library(crowd_simulator_common
  SHARED
  src/crowd_simulator_common.cpp  
)

target_include_directories(crowd_simulator_common
  PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${menge_INCLUDE_DIRS}
    ${IGNITION-MATH_INCLUDE_DIRS}
)

target_link_libraries(crowd_simulator_common
  PUBLIC
    ${GAZEBO_LIBRARIES}
    ${menge_LIBRARIES}
)

###############################
# install stuff
###############################
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)
ament_export_libraries(crowd_simulator_common)

install(
  TARGETS crowd_simulator_common
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

ament_package()
