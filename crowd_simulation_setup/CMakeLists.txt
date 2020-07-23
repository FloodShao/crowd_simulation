cmake_minimum_required(VERSION 3.5)
project(crowd_simulation_setup)

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

install(
  DIRECTORY config_resource/
  DESTINATION share/${PACKAGE_NAME}
)

ament_package()


set(config_resource_dir ${CMAKE_SOURCE_DIR}/config_resource)

file(GLOB_RECURSE menge_setup_files "config_resource/*.menge_setup.yaml")
file(GLOB_RECURSE traffic_editor_files "config_resource/*.menge_navmesh.building.yaml")

foreach(traffic_editor_file ${traffic_editor_files})

  # get the world name
  string(REPLACE "." ";" list1 ${traffic_editor_file})
  list(GET list1 0 name)
  string(REPLACE "/" ";" list2 ${name})
  list(GET list2 -1 world_name)

  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/${project_name}/config_resource/${output_world_name})

  add_custom_command(
    OUTPUT ${output_world_name}
    COMMAND ros2 run building_navmesh_tools navmesh_generator ${traffic_editor_file} ${output_dir} ${output_world_name}
    DEPENDS ${traffic_editor_file}
  )

  add_custom_target(
    generate_${output_world_name}_navmesh ALL
    DEPENDS ${output_world_name}
  )

endforeach()

# generate menge config file, add plugin part to the world file
foreach(menge_setup_file ${menge_setup_files})

  # get the world name
  string(REPLACE "." ";" list1 ${menge_setup_file})
  list(GET list1 0 name)
  string(REPLACE "/" ";" list2 ${name})
  list(GET list2 -1 world_name)

  set(menge_config_file ${menge_setup_file})
  set(output_world_name ${world_name})
  set(output_menge_dir ${CMAKE_CURRENT_BINARY_DIR}/config_resource/${output_world_name})
  set(temp_target ${world_name}_config)

  set(map_path ${config_resource_dir}/${output_world_name}/${output_world_name}.map.building.yaml)
  set(output_maps_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})
  set(output_world_path ${output_maps_dir}/${output_world_name}.world)
  set(output_model_dir ${output_maps_dir}/models)
  
  # todo, add check map_path exists

  # generate world file
  set(no_download_flag "")
  if (NO_DOWNLOAD_MODELS)
    set(no_download_flag "--no_download")
  endif()

  add_custom_command(
    OUTPUT  temp_target
    COMMAND ros2 run building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir} ${no_download_flag}
    COMMAND ros2 run building_navmesh_tools configfile_generator ${menge_config_file} ${output_dir} gazebo ${output_world_path}
    DEPENDS ${map_path}
  )

  add_custom_target(
    generate_${output_world_name}_configfile ALL
    DEPENDS temp_target
  )

  install(
    DIRECTORY ${output_menge_dir}
    DESTINATION share/${PROJECT_NAME}/config_resource
  )

  install(
    DIRECTORY ${output_maps_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )

endforeach()
