# Crowd simualtion plugin for gazebo and ignition-gazebo

## Reference:
**Menge**: <https://github.com/MengeCrowdSim/Menge>. Menge is an open framework for crowd simulation. In this simulation, the original menge lib is modified to satisfy the scenario for rmf project.

## Environment Setup
1. Please use the following ignition source branch, if you are going to use ignition-gazebo plugin.
```
repositories:
  ign-cmake:
    type: git
    url: https://github.com/ignitionrobotics/ign-cmake
    version: ign-cmake2
  ign-common:
    type: git
    url: https://github.com/ignitionrobotics/ign-common
    version: ign-common3
  ign-fuel-tools:
    type: git
    url: https://github.com/ignitionrobotics/ign-fuel-tools
    version: ign-fuel-tools4
  ign-gazebo:
    type: git
    url: https://github.com/ignitionrobotics/ign-gazebo
    version: master
  ign-gui:
    type: git
    url: https://github.com/ignitionrobotics/ign-gui
    version: master
  ign-launch:
    type: git
    url: https://github.com/ignitionrobotics/ign-launch
    version: master
  ign-math:
    type: git
    url: https://github.com/ignitionrobotics/ign-math
    version: ign-math6
  ign-msgs:
    type: git
    url: https://github.com/ignitionrobotics/ign-msgs
    version: ign-msgs5
  ign-physics:
    type: git
    url: https://github.com/ignitionrobotics/ign-physics
    version: ign-physics2
  ign-plugin:
    type: git
    url: https://github.com/ignitionrobotics/ign-plugin
    version: dlopen_exception
  ign-rendering:
    type: git
    url: https://github.com/ignitionrobotics/ign-rendering
    version: master
  ign-sensors:
    type: git
    url: https://github.com/ignitionrobotics/ign-sensors
    version: master
  ign-tools:
    type: git
    url: https://github.com/ignitionrobotics/ign-tools
    version: ign-tools1
  ign-transport:
    type: git
    url: https://github.com/ignitionrobotics/ign-transport
    version: ign-transport8
  sdformat:
    type: git
    url: https://github.com/osrf/sdformat
    version: sdf9
```
2. Manually modify `Actor.cc` file in sdformat9. Please check the PR for `Actor.cc` in <https://github.com/osrf/sdformat/pull/301/files>. This PR had been merged into sdf8 but may not be merged into sdf9 yet. If not, you will need to manually modify all the copy constructor in Actor.cc.

3. Menge dependencies:
The core lib for meng is at <https://github.com/FloodShao/menge_core.git> to make menge possible be compiled with colcon.

## Steps:
1. Compile ignition package in work space `ignition_ws` after modified the `Actor.cc` in sdf9
2. In crowd simulation work space `dev_ws`. Download the repositories file "crowd_simulation_plugin.yaml" in the repo, and proceed:
```
vcs import src < ./crowd_simulation_plugin.yaml
```
3. Colcon compile packages as following order:
```
colcon build --packages-select menge
colcon build --packages-select crowd_simulation_common
colcon build --packages-select crowd_simulation_gazebo
colcon build --packages-select crowd_simulation_ign
```

## Run simulation
1. source environment
```
source ~/ignition_ws/install/setup.bash
source ~/dev_ws/inatll/setup.bash
```
2. for gazebo plugin
```
cd ~/dev_ws/src/crowd_simulation_gazebo/test/crowd_simulation/show_and_tell/
ros2 launch office_launch.launch
```
3. for ignition-gazebo plugin
```
cd ~/dev_ws/src/crowd_simulation_ign/test/crowd_simulation/test_world/
ros2 launch test_world.launch.xml
```