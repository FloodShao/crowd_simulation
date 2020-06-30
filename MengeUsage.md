## Menge library used in crowd simulation

Menge supports a few strategies for crowd simulation. In order to better support the auto-generation of navmesh file. There are a few points need to be consider when dealing with menge configuration file: behavior.xml, scene.xml, and navmesh.nav.

### Scene File
Scene file defines the crowd simulation environment. Basically it defines the action for two sub-problem:
* path-plan
* collision avoidance

1. SpatialQuery
`<SpatialQuery>` defines the strategy in path-plan. In this specific situation, the path-plan strategy should be configured as "kd-tree" instead of "navmesh". Because the auto-generation navmesh shows a clear connection relationship between nodes, and thus "kd-tree" handles this well. In "navmesh", when finding a certain path, the agent will check whether the target is visible from current position, which may results in a "failure of finding a path to goal" problem.

**The correct way:**
`<SpatialQuery type="kd-tree" test_visibility="false" />`

2. ObstacleSet
`<ObstacleSet>` defines the where the obstacles are. Rather than configuring obstacles manually, the obstacle line is actually the non-connected edge in the auto-generation navmesh. Therefore, this should be configured as navmesh.

**The correct way:**
`<ObstacleSet class="1" file_name="test_navmesh.nav" type="nav_mesh" />`

3. Agents
There are two types of agents in the crowd simulation: internal agents and external agents. Specifically, internal agents are the "crowds" that controlled by menge simulator. External agents are static or non-static obstacles in the world file, for example, static obstacles for small drawer etc., and non-static obstacles for moving robots. 

The logic for this two types agents is menge will treat external agents as static, and menge will not apply actions on external agents, rather menge will only decides the moving action for internal agents. For every single simulation round, the position of external agents (static or non-static obstacles) will be updtaed to menge, and menge will calculate the moving action for internal agents.

**For external agents:**
All the external agents are treated equally. Several key points:
(1) in AgentProfile/Common tag, `pref_speed="0"`, and `r="0.2"`. These two significantly affect the collision avoidance.
(2) in AgentGroup/Generator tag, the number of `<Agent>` should matchh the number of `<external_agent>` in the crowd_simulator plugin in world.sdf.
```xml
    <AgentProfile name="static_obstacle">
      <Common class="1" max_accel="5" max_angle_vel="360" max_neighbors="10" max_speed="2" neighbor_dist="5" obstacleSet="1" pref_speed="0" r="0.2" />
      <PedVO buffer="0.9" factor="1.57" tau="3" tauObst="0.1" turningBias="1.0" />
      <ORCA tau="1.0" tauObst="0.4" />
    </AgentProfile>

    <AgentGroup>
      <ProfileSelector name="static_obstacle" type="const" />
      <StateSelector name="static" type="const" />
      <Generator type="explicit">
        <Agent p_x="18.83" p_y="-3.85"/>
        <Agent p_x="18.83" p_y="-3.85"/>
        <Agent p_x="18.83" p_y="-3.85"/>
      </Generator>
    </AgentGroup>
```

**For internal agents:**
Similar as above external agents. The key point is the position of Agent should be within the navmesh. Not on the edge, not on the obstacle, but within the defined node area.

### Behavior File
Behavior file defines the a finite state machine for each agent, including the state trasitions and goal selection.

The typical state definition include:
```xml
<State final="0" name="walk">
    <GoalSelector goal_set="0" type="weighted" />
    <VelComponent file_name="test_navmesh.nav" heading_threshold="15" type="nav_mesh" />
</State>
<Transition from="walk" to="walk">
    <Condition distance="0.05" type="goal_reached" />
</Transition>
<GoalSet id = "0">
    <Goal capacity="2" id="0" type="point" weight="1" x="18.83" y="-4" />
</GoalSet>
```

The key point is: make sure each `<Goal>` in `<GoalSet>` should be within the defined navmesh. Not on the edge, not on the obstacle, but within the nodes defined in navmesh.

**For external agents:** external agents may occurred anywhere, it may not be necessarily within the navmesh. (Because navmesh defines where the agents can go, external agents may not be necessarily appearred within the navmesh for internal agents.) As such, in order to make external agents be able to appearred anywhere, the `<VelComponent>` in `<State>` for external agents should be a different navmesh. The most simple one is a rectangle area with only one node that covers the entire area of the map. 
Besides, external agents should remain within the same state because these agents are not supposed to be controlled by agents.

