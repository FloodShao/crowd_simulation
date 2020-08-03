# crowd_simulation_setup package documentation
## Intro
crowd_simulation_setup is a package generating all necessary files for crowd simulation plugins in gazebo environment based on the open source library [Menge](https://github.com/MengeCrowdSim/Menge). A modification menge library which introduces external agents and fixing some C++ standard problem can be found in [menge_core](https://github.com/FloodShao/menge_core)

## Logic
The package has the directory of `config_resource`, which contains the yaml files defining how the crowd simulation will process. Currently, there is an example for office world `config_resource/office`:
* `office.map.building.yaml`: This is the exact same file in package `rmf_demo_maps` for generating the original office world file
* `office.menge_navmesh.building.yaml`: This is the yaml file generated from the modified version of `traffic-editor` gui <https://github.com/FloodShao/traffic_editor/tree/fred/human_lanes> (in branch: fred/human_lanes). Human lanes are defined in this yaml file, and the vertices for spawning robots are included as well. **The navmesh file will be generated based on this file.**
* `office.menge_setup.yaml`: This is a yaml file to configure the menge setups, for the generation of **behavior file** and **scene file**. The `<plugin>` tag will also be configured in this yaml file.
* `office.png` and `office.project.yaml` is used for `traffic-editor`.

The package will generate all the required files during compiling period. This python command part is called in CMakeLists.txt. 

**[Note]** :
The bug that the world file is generated without inserting the <plugin> tag has been fixed.

## menge setup files explanation
When generating the navmesh file from `office.menge_navmesh.building.yaml` , a template menge setup file will also be generated as follows:

```yaml
L1:
 goals:
  - [2.784419793478699, -9.068542907528862, 0.0, conf2]
  - [6.925883817832386, -2.058469742796842, 0.0, store]
  - [9.066477326770338, -2.6844930915376977, 0.0, store]
  - [9.975214343591647, -3.5730399029939375, 0.0, common]
  - [13.105331087295925, -3.6740078608091125, 0.0, common]
  - [15.932518561069944, -3.7345954078941466, 0.0, common]
  - [18.71932562411382, -3.7547856132592172, 0.0, common]
  - [18.7395242949738, -5.572268112396749, 0.0, common]
  - [18.759714500338873, -6.743537057428356, 0.0, common]
  - [18.779913171198857, -8.92451944259136, 0.0, common]
  - [18.759714500338873, -10.459279808153525, 0.0, common]
  - [17.48747759749209, -11.04491428066933, 0.0, common]
  - [15.16513837828886, -11.105493362259452, 0.0, common]
  - [13.388044755376379, -11.125692033119433, 0.0, common]
  - [11.651340008688953, -11.105493362259452, 0.0, common]
  - [10.399293311207241, -11.509382124509976, 0.0, common]
  - [7.410550332533015, -10.681414394643857, 0.0, common]
  - [8.117342968229064, -8.92451944259136, 0.0, common]
  - [7.4913280849831185, -8.157130794315364, 0.0, common]
  - [8.581810812069708, -6.036735956237396, 0.0, common]
  - [7.633429882575601, -5.614003002312806, 0.0, conf2]
  - [5.351479539597422, -4.503321604366234, 0.0, conf2]
  - [5.290891992512388, -10.238482769859285, 0.0, conf2]
  - [16.902588088528542, -5.210114240062283, 0.0, common]
  - [15.165883341841115, -6.845851028934535, 0.0, common]
  - [10.177903688269163, -6.886239905159588, 0.0, common]
  - [11.571307219791102, -6.946818986749711, 0.0, common]
  - [19.891339532697682, -3.2916637831095756, 0.0, conf1]
  - [22.698336801106628, -5.6341932076778765, 0.0, conf1]
  - [23.22339219203231, -3.1301082782093665, 0.0, conf1]
  - [6.1092175236737605, -3.50961641511339, 0.0, store]
 state:
  - {name: , goal_set: , navmesh_file_name: , final: 0}
 transition:
  - {from: , to: , Condition: , Target: }
 goal_set:
  - {set_id: , set_area: , capacity: }
 goal_area:
  - conf1
  - conf2
  - common
  - store
 obstacle_set:
  - {class: obstacleSetId, file_name: , type: nav_mesh}
 agent_profile:
  - {name: , class: agentClassId, max_accel: 5, max_angle_vel: 360, max_neighbors: 10, max_speed: 0, neighbor_dist: 5, obstacleSet: obstacleSetId, pref_speed: 0, r: 0.2, ORCA_tau: 1.0, ORCA_tauObst: 0.4}
 agent_group:
  - {group_id: , profile_selector: , state_selector: }
 agent_list:
  - {group_id: 0, agents_number: , agents_name: ['magni2', 'magni1'], x: 0.0, y: 0.0}
model_type:
 - {type_name: , animation_speed: , animation_file: , gazebo: {'filename': '', 'initial_pose': [0, 0, 0, 0, 0, 0]}, ign: {'model_file_path': '', 'initial_pose': [0, 0, 0, 0, 0, 0]} }
```

All the parameters listed are either default value or retrieved from the `office.menge_navmesh.building.yaml`. 
### Behaviour file related
Behaviour file defines the finite behaviour state machine that controls how the agent will schedule his route. This is the general problem of crowd simulation.
1. `goals`. Goals represent the heading target for each agent. In current version, each goal is the vertex with a specific name but without extra properties in `office.menge_navmesh.building.yaml`. When creating the `office.menge_navmesh.building.yaml` in `traffic-editor` GUI, the goals are clustered by the actual location. For example, all the goals within the conference room2 will be named as "conf2". **Note:** each goal must be within the constructed navmesh, and to guarantee this requirement, the goal is selected as the intersection vertices.
2. `state`, generated for menge behaviour configuration. 
    ```
    {name: 'common_walking', goal_set: 0, navmesh_file_name: 'L1_navmesh.nav', final: 0}
    ```
3. `transition`, generated for menge behaviour configuration. A transition defines at what condition, a transition will occur to the target state. One transition consists of `from` state, `to` state, `Condition`, and `Target` state. The `Condition` represents a boolean condition. When the `Condition` is true, the transition will occur. The `Target` represents a set of `to` states, making it possible for agents to transit to other states. 
    1. `Condition` in menge has following type, and you can assemble any type to one condition:
        1. `goal_reached` type. When the agent reaches the assigned target within a certain distance, the condition is true.
        ```
        {type: goal_reached, distance: 0.2}
        ```
        2. `timer` type. When the agent stays in the current state for a certain duration, the condition is true.
        ```
        # for constant duration configuration
        {type: timer, dist: 'c', value: 30.0, per_agent: true}
        # for random duration selection (uniform distribution)
        {type: timer, dist: 'u', min: 10.0, max: 100.0, per_agent: true}
        ```
        3. `and` type or `or` type. Boolean calculation for two conditions.
        ```
        {type: and, condition1: {}, condition2: {}}
        {type: or, condition1: {}, condition2: {}}
        ```
        4. `not` type
    2. `Target` represents the set of `to` states. When the `Condition` is met, the `to` state will selected from the `Target` set based on the weighted probability:
        ```
        Target: [
         {name: 'store_walking', weight: 0.3}, 
         {name: 'common_walking', weight: 1.0}
        ]
        ```
4. `goal_set` defines the target goals set for each state. (Each state has an attribute of "goal_set"). For each goals set, a unique set_id will be assigned, which must be consistent with the goal_set id in the corresponding state. The `set_area` includes all the goals within the target areas. The `capacity` represents the maximum number of agents who select the same goal as the current target. 
    ```
    {set_id: 0, set_area: {common}, capacity: 2}
    ```
5. `goal_area` is auto generated from `goals`, basically counting all the unique goal name.
### Scene file related
Scene file defines the local problem of crowd simulation, including the collision avoidance, spawning agents and initial state for agents.
1. `obstacle_set` defines the obstacle walls that agents should not penetrate. Usually, we can use the generated navmesh.
2. `agent_profile` defines the agent properties such as the speed, acceleration, and the collision area. In menge, each agent is modelled as a circle, the radius is defined by the `r` attribute. 
    ```
    {name: 'external_agent', class: 1, max_accel: 0, max_angle_vel: 0, max_neighbors: 10, max_speed: 0, neighbor_dist: 5, obstacleSet: 1, pref_speed: 0, r: 0.25, ORCA_tau: 1.0, ORCA_tauObst: 0.4}
    ```
3. `agent_list` defines how many agents are spawned. There are two type of agents (external agents represent any models we want the crowd to avoid such as the moving robot, static furniture, etc.). Under a careful designed navmesh, we don't necessarily set the static model as the external agents. What really matters is the moving robot. In office world, the robot is spawned from two vertices with property of `spawn_robot_name`. As such, the external agents is listed when processing the `office.menge_navmesh.building.yaml`. (For external agents, the initial position x and y is not important). The external agents must be assigned with '0' group_id, because when spawning agents, the external agents will be first processed.
    ```
    # for external agents
    {group_id: '0', agents_number: , agents_name: ['magni2', 'magni1'], x: 0.0, y: 0.0}
    # for internal agents to be spawned, the initial position x and y must be within the navmesh area
    {group_id: '1', agents_number: 5, agents_name: , x: 18.71932562411382, y: -3.7547856132592172}
    ```    
### Plugin part related
`model_type` defines the model skeleton when spawning internal agents. The model type name must be consistent with the `name` attribute in `agent_profile`.  

```
- {typename: 'human',
 animation_speed: 0.2,
 animation: 'walk',
 gazebo: {filename: walk.dae, initial_pose: [0, 0, 0, 0, 0, 0]},
 ign: {model_file_path: https://fuel.ignitionrobotics.org/1.0/Mingfei/models/actor, initial_pose: [0, 0, 0, 0, 0, 0]}
}
```
    

