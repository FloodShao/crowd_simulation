source ~/lib_ws/install/setup.bash
source ~/dev_ws/install/setup.bash

export IGN_FILE_PATH=${PWD}/../models:/usr/share/gazebo-9/models
export SDF_PATH=${PWD}/../models:${PWD}/test.world
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/headless/dev_ws/install/building_sim_ign/lib
export MENGE_RESOURCE_PATH=${PWD}

