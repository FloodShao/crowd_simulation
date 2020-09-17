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
Crowd simulation can be defined into 2 sub-problems: global path plan, and local collision avoidance. In Menge, the global path plan is solved by constructing a connection graph (navmesh) that defines where the human is able to reach, and a finite state machine (FSM) to drive the human keep switching the heading target position. The local collision avoidance is solved under the ORCA (optimal reciprocal collision avoidance), in which, each human is modeled as a circle (with a configurable radius). The configuration of this 2 sub-problems are integrated in the traffic-editor gui.

### Navmesh (connection graph)
#### Human Lanes
The detailed introduction of navmesh (Menge is its own navmesh definition) can be found in [to be updated](). Basically, navmesh defines lots of nodes (geometrically convex polygon) connected with each other with edges (geometrically the common edge between adjecent nodes), and some obstacles (geometrically another type of edge with only one node).
For a generalization purpose, in crowd simulation, the navmesh is generated from the human lanes with predefined width. [to be updated]()

In traffic-editor, the human lanes can be edited under the *Crowd_Sim* Edit mode by clicking the *add human lane* button. The "edge_type" (defined in traffic_editor gui) of human lanes is "human_lane" with default "graph_idx" of 9 (make sure you are in the right graph_idx when select the human lanes). Additional property of "width" is added in the edge params with default value of 1.0. You can easily change the width of the human lane, the width of the lane drawing will change accordingly. 

![traffic_editor_human_lane](https://github.com/FloodShao/crowd_simulation/blob/master/crowd_simulation_doc/figs/traffic_editor_human_lane.png?raw=true)

**Note**:
1. You should make each human lane connected to the whole graph. That means at least 2 human lanes should be defined, and no orphan human lane is expected.
2. If you check the navmesh generation logic, the polygon vertices are generated based on the intersection lanes. So the following situation is not expected.

![unexpected_lane_crossing](https://github.com/FloodShao/crowd_simulation/blob/master/crowd_simulation_doc/figs/unexpected_lane_crossing.png?raw=true)

#### Human Goals
In Menge, each heading goal should be within the navmesh graph. Menge implements this [point within a polygon check](https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/) to check whether the heading target is accessible within the navmesh. If the target is outside the navmesh, Menge will be dumped with an error ("Could not find the target inside the navmesh. Bad navmesh..."). Luckily, when constructing the navmesh, the lane vertices of the human lanes must be within the navmesh. Therefore, in the crowd simulation configuration, lane vertices are assigned as the heading targets. All the vertices in the traffic-editor scene should be accessed under the *Building* edit mode. A property for vertice "human_goal_set_name" can be added, and set with a string name. If a lane vertice does not have a "human_goal_set_name" property, it will not be assigned as a target position in the FSM.

In Menge, a goal set includes multiple goals, and a goal set is related with a state. In order to make it easy to configure the state, it is suggested to name the "human_goal_set_name" as location name. For example, there are multiple lane vertices in the conference room, if you want "reach the conference" to be a human state, it is suggested to name the "human_goal_set_name" for all lane vertices in the conference room as "conf", so that when the human is set to "reach the conference" state, the human will randomly select one of the target named with "conf" rather than reaching to a constant position everytime except that's what you want.

![human_goals](https://github.com/FloodShao/crowd_simulation/blob/master/crowd_simulation_doc/figs/human_goals.png?raw=true)

### FSM configuration (Global path plan)
A *crowd_sim* panel is created in traffic-editor. You can define the configuration of the crowd simulation in this panel. Make sure you have updated all the human lanes and the "human_goal_set_name" for all the lane vertices first.
![crowd_sim_panel](https://github.com/FloodShao/crowd_simulation/blob/master/crowd_simulation_doc/figs/crowd_sim_panel.png?raw=true)

#### enable_crowd_sim
If you uncheck this, no "crowd_simulation" plugin will be inserted in the world.sdf. No crowd simulation will be performed under this situation.

#### update_time_step
This is the simulation time step for Menge. If you set the update_time_step smaller, it is likely to decrease the RTF for gazebo and ign simulation. If you set the update_time_step bigger, it is likely you will see the jumping animation of the human. The default setting is 0.1, which is acceptable for RTF and animation.

#### GoalSets
**GoalSets is the basis of the whole crowd simulation setup. Please configure GoalSets first!** In this setup, you will create multiple goal sets labeled with a goal set id. Traffic-editor will collect all the "human_goal_set_name" you defined in the traffic-editor scene, and create a multi-selection checkbox for you to choose. For example, if you have "conf", "table_area", "supplies", and "common" goal set area, you can create a goal set with id = 0, and check "conf" and "table_area" to make the goals within two areas in one goal set.

**Note**:
A goal set without checking any area is invalid goal set, and this configuration will not be saved.

#### States
**Please configure GoalSets first.** State is the basic element of the FSM. There are 4 elements to be set: "name" (specifying the state name), "is final" (state transition ends with final state), "navmesh_file" (select from the navmesh will be generated, for now, each level has one navmesh), "goal_set" (specifying the goal_set id, select from the combo box) 
One state is attached with a single goal set. When the human is at the current state, one of the human goals will be randomly selected as the current human heading target. 
A default final state "external_static" is set and not allowed to change. This state is prepared for external agents like moving robots. 

#### Transitions
**Please configure States first.** Transition defines the transitions between states when certain condition is met. Each transition starts from one "from state", ends with a set of "to state set" when "Condition" is met.
1. "from state" is required for a transition, otherwise, it's an invalid transition.
2. "to state set" is required for a transition. If there are multiple to_state set in the "to state set", the transition will randomly choose one of them as actual to_state based on the weight each to_state assigned.
3. "Condition" is required for a transition.
There are 2 basic condition "goal_reached" and "timer" provided in Menge. (For "timer", menge supports const timer, and random duration timer. Only const timer is intergrated in traffic-editor for now) There are 3 condition calculations "and", "or", and "not" also provided in Menge. For basic condition, a value is configured. "goal_reach_distance (m)" is for "goal_reached" condition, which makes the condition met when the distance between human and the current heading target is less than teh "goal_reach distance". "Duration (s)" is for "timer" condition, which makes the condition met if the human has been in this state for the duration time. **Note: the timer starts when the human transits to the current state. At this time, the human may not reach the current target yet.**
Theoretically, a condition tree can be constructued with the three condition calculations. However, for simplicity, only one condition calculation is allowed for the root condition.

**Upon finish the above setup, a FSM required for the global path plan problem is constructed.** In menge, once the human transits to a certain state, and assigned with a target position, A* algorithm will be used to generate the velocity map to guide the human to reach the target position.

### Local collision avoidance
Each human is modeled as a circle in Menge. The following setup is to set up the local behavior of the human.
#### AgentProfiles
One agent profile defines one type of human (with same circle radius, same collision avoidance setup, same walking speed, **using the same actor skeleton**).
There is one default agent profile for external agent (for moving robots). All the speed are set to 0, because in Menge, external agents are not supposed to be moving, while external agent position is directly update from the simulation world. As such, the only parameter you might need to set for external agent is the radius "r" (according to different robot size)

Other agent profile, you can specifying the walking speed, radius, and other parameters. For walking human, the following parameters are suggested:
* max_accelration = 5
* max_angle_velocity = 360
* max_neighbors = 5 (the maximum number of collision avoid interaction agents)
* max_speed = 2 (walking to front)
* neighbor_dist = 5 (neighbors less than the dist is ready for ORCA interaction)
* pref_speed = 1.5
* r = 0.25
* ORCA_tau = 1 (if collision with agents happens within the future 1s, motion planning triggers)
* ORCA_tauObst = 0.4 (if collision with obstacle, usually walls, happens within the future 1s, motion planning triggers)
Please leave the "class" and "obstacle_set" as default 1.

Reduce the max_neighbors and neighbor_dist might benefit the simulation RTF, because the number of negotiation process in Menge will be reduced. The benefit is not guaranteed, because the RTF is affected by many factors.

### Agent spawn related
#### AgentGroup
In our modification of Menge, there are 2 types of agents: external agents and internal agents.
External agents are typically moving robots, while internal agents are the actual crowds. The crowd simulation plugin updates the position of 2 types of agents differently as follows: for external agents, plugin get the robot position from the simulation world and update in Menge, while for internal agents, plugin get the human position from the Menge simulation result to the gazebo simulation world.

![plugin_structure](https://github.com/FloodShao/crowd_simulation/blob/master/crowd_simulation_doc/figs/plugin_structure.png?raw=true)

1. External Agent
**Traffic-editor** is assuming all the robots are spawned from the "spawn_robot_name" defined in the "building.yaml". Traffic-editor will traverse all the vertices that has "spawn_robot_name" property, and add the robot name to the external agent group. Please do not change anything in the external agent.

2. Internal Agent, agents are spawned in groups.
* Each group are spawned at the same position "x, y". The position (x, y) is suggested to configured at one of the human goals. Mind that you should get the transformed point vertices in the traffic-editor scene.
* Each group spawns agents under same agent profile and starts the agents with the same State.
* You will just need to specify the number of agents you want to spawn for this group.

#### ModelType
This is related to the actor skin and animation to be played in the gazebo and ign gazebo.
* "name" should be consistent with the name of desired AgentProfile type.
* "animation" should be specified as "walk" or "walking" that is attached in the "filename"
* "animation_speed" is the value to adjust the play speed of the animation to be able to synchroinze the animation with walking distance.
For gazebo: (Currently gazebo might have different actor spawn method compared to ign gazebo)
* "gazebo_model" should be specified as the .dae file like "walk.dae" or "model://MaleVisitorPhone/meshes/MaleVisitorPhoneWalk.dae"
* "gazebo_idle" is reserved for the future animation switch feature. It should be set as the .dae file like "stand.dar" or "model://MaleVisitorPhone/meshes/MaleVisitorPhoneIdle.dae"
* "x, y, z, pitch, roll, yaw" is set as the initial coordinate system of the actor. You might need to adjust these if the actor is hanged in the air or facing the ground.
For ign: (ign gazebo can easily spawn the actors using a service call)
* "ign_model": can be either the html address in the ignition fuel, or local directory "model://MaleVisitorPhone". ign gazebo will load the model.sdf in the model directory.
* "x, y, z, pitch, roll, yaw" is same as gazebo

**Upon finish all these steps, save the building, all the setup will be saved to building.yaml. The next step is to transform the yaml setup to the .xml format and .nav format that required for Menge**
