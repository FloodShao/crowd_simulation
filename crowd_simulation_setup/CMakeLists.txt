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
find_package(rmf_demo_maps REQUIRED)

install(
  DIRECTORY config_resource/
  DESTINATION share/${PACKAGE_NAME}
)

ament_package()

include(GNUInstallDirs)

file(GLOB_RECURSE menge_setup_files "config_resource/*.menge_setup.yaml")
file(GLOB_RECURSE traffic_editor_files "config_resource/*.menge_navmesh.building.yaml")
file(GLOB_RECURSE map_files "maps/*.building.yaml")


# from rmf_demo_maps
foreach(path ${map_files})

  # Get the output world name
  string(REPLACE "." ";" list1 ${path})
  list(GET list1 0 name)
  string(REPLACE "/" ";" list2 ${name})
  list(GET list2 -1 world_name)

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})
  set(output_world_path ${output_dir}/${output_world_name}.world)
  set(output_model_dir ${output_dir}/models)

  # first, generate the world
  set(no_download_flag "")
  if (NO_DOWNLOAD_MODELS)
    set(no_download_flag "--no_download")
  endif()
  add_custom_command(
    OUTPUT ${output_world_path}
    COMMAND ros2 run building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir} ${no_download_flag}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name} ALL
    DEPENDS ${output_world_path}
  )

  # now, generate the nav graphs
  set(output_nav_graphs_dir ${output_dir}/nav_graphs/)
  set(output_nav_graphs_phony ${output_nav_graphs_dir}/phony)
  add_custom_command(
    OUTPUT ${output_nav_graphs_phony}
    COMMAND ros2 run building_map_tools building_map_generator nav ${map_path} ${output_nav_graphs_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name}_nav_graphs ALL
    DEPENDS ${output_nav_graphs_phony}
  )

  install(
    DIRECTORY ${output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )

endforeach()


# generate navmesh
foreach(traffic_editor_file ${traffic_editor_files})

  # get the world name
  string(REPLACE "." ";" list1 ${traffic_editor_file})
  list(GET list1 0 name)
  string(REPLACE "/" ";" list2 ${name})
  list(GET list2 -1 world_name)

  set(menge_navmesh_path ${traffic_editor_file})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/config_resource/${output_world_name})
  set(temp_target ${world_name}_navmesh)
  
  add_custom_command(
    OUTPUT ${temp_target}
    COMMAND ros2 run building_navmesh_tools navmesh_generator ${traffic_editor_file} ${output_dir} ${output_world_name}
    DEPENDS ${traffic_editor_file}
  )
  
  add_custom_target(
    generate_${temp_target} ALL
    DEPENDS ${temp_target}
  )

  file(GLOB_RECURSE navmesh_files ${output_dir}/*.nav)
  install(
    FILES ${navmesh_files}
    DESTINATION share/${PROJECT_NAME}/config_resource
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
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/config_resource/${output_world_name})

  set(original_world_file ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name}/${output_world_name}.world)
  set(output_dir_world ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})

  set(temp_target ${world_name}_config)

  add_custom_command(
    OUTPUT ${temp_target}
    COMMAND ros2 run building_navmesh_tools configfile_generator ${menge_config_file} ${output_dir} gazebo ${original_world_file}
    DEPENDS ${menge_config_file}
  )

  add_custom_target(
    generate_${output_world_name}_configfile ALL
    DEPENDS ${temp_target}
  )

  file(GLOB_RECURSE config_files ${output_dir}/*.xml)
  install(
    FILES ${config_files}
    DESTINATION share/${PROJECT_NAME}/config_resource
  )

endforeach()





