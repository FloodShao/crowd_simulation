## Tips to use ignition plugin

1. use the following ignition source branch:
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

2. for sdf9, check the PR for `Actor.cc` in <https://github.com/osrf/sdformat/pull/301/files>

this PR had been merged into sdf8 but not into sdf9 yet. You need to manually modify all the copy constructor in Actor.cc.

3. menge dependencies:
in <https://github.com/FloodShao/menge_core.git>