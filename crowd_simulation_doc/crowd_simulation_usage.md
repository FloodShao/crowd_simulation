# Crowd simulation in RMF demo usage manual

## Introduction
Welcome to crowd simulation setup for RMF demo. In this setup, the [Menge Library](https://github.com/MengeCrowdSim/Menge) is used, and the Menge core is extracted and added with a CMakeLists.txt in <https://github.com/FloodShao/menge_core> to make it possible to build Menge together with RMF.

A typical repos list for using the crowd simulation in RMF (after the crowd simulation integration is merged) is listed as follow:
```
repositories:
  rmf/rmf_core:
    type: git
    url: https://github.com/osrf/rmf_core.git
    version: master
  rmf/rmf_schedule_visualizer:
    type: git
    url: https://github.com/osrf/rmf_schedule_visualizer.git
    version: master
  rmf/traffic_editor:
    type: git
    url: https://github.com/osrf/traffic_editor.git
    version: master
  rmf/rmf_demos:
    type: git
    url: https://github.com/osrf/rmf_demos.git
    version: master
  meng_core:
    type: git
    url: https://github.com/FloodShao/menge_core.git
    version: master
```

You can check the following PRs for the related review on the crowd simulation integration:
* [Crowd simulation plugin with gazebo and ign-gazebo](https://github.com/osrf/traffic_editor/pull/218)
* [Crowd simulation navmesh and configureation files generation](https://github.com/osrf/traffic_editor/pull/224)
* [Crowd simulation traffic-editor gui integration](https://github.com/osrf/traffic_editor/pull/225)

## Crowd simulation work pipeline
Crowd simulation can be defined into 2 sub-problems: global path plan, and local collision avoidance. In Menge, the global path plan is solved by constructing a connection graph (navmesh) that defines where the human is able to reach, and a finite state machine to drive the human keep switching the heading target position. The local collision avoidance is solved under the ORCA (optimal reciprocal collision avoidance), in which, each human is modeled as a circle (with a configurable radius). The configuration of this 2 sub-problems are integrated in the traffic-editor gui.

### Navmesh (connection graph)
#### Human Lanes
The detailed introduction of navmesh (Menge is its own navmesh definition) can be found in [to be updated](). Basically, navmesh defines lots of nodes (geometrically convex polygon) connected with each other with edges (geometrically the common edge between adjecent nodes), and some obstacles (geometrically another type of edge with only one node).
For a generalization purpose, in crowd simulation, the navmesh is generated from the human lanes with predefined width. [to be updated]()

In traffic-editor, the human lanes can be edited under the *Crowd_Sim* Edit mode by clicking the *add human lane* button. The "edge_type" (defined in traffic_editor gui) of human lanes is "human_lane" with default "graph_idx" of 9 (make sure you are in the right graph_idx when select the human lanes). Additional property of "width" is added in the edge params with default value of 1.0. You can easily change the width of the human lane, the width of the lane drawing will change accordingly. 
![]()

**Note**:
1. You should make each human lane connected to the whole graph. That means at least 2 human lanes should be defined, and no orphan human lane is expected.
2. If you check the navmesh generation logic, the polygon vertices are generated based on the intersection lanes. So the following situation is not expected.

#### Human Goals
In menge, 

