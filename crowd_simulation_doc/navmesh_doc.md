## Auto generation of navmesh for menge crowd simulation 

In menge, navmesh can be used for agents path planing. Navmesh file defines the accessible area of agent, in the meantime, navmesh file defines obstacle areas. Navmesh file is used in both **behavior** file and **scene** file. 
In **behavior** file, the finite state machine is defined. Navmesh is used to calculate the velcomponent for each agent in the current state. For example:

```xml
<State final="0" name="walk">
    <GoalSelector goal_set="0" type="weighted" />
    <VelComponent file_name="test_navmesh.nav" heading_threshold="15" type="nav_mesh" />
</State>
<Transition from="walk" to="walk">
    <Condition distance="0.05" type="goal_reached" />
</Transition>
```
In **Scene** file, the obstalce sets can be defined using navmesh. In general, navmesh can be seen as a k-dimensional tree which defines the connection relationship between each location. As such, in scene file, using "kd-tree" as spatial query is better for navmesh. For example:
```xml
<SpatialQuery type="kd-tree" test_visibility="false" />
<Common time_step="0.1" />
<ObstacleSet class="1" file_name="test_navmesh.nav" type="nav_mesh" />
```

### Navmesh Elements
Navmesh used in menge is a simple text file that can be divided into following parts:

#### Vertex
Vertex element is the basic element for navmesh, all the other elements are constructed based on vertices. Vertex part start with the number of vertices defined, then following the 2D coordinates for each vertex. The vertices is numbered starting from 0. For example:
```
## format
# [Number]
# [X0] [Y0]
# [X1] [Y1]
# ......

4
18.320185574061778 -4.6 
19.33330032086155 -3.5999575646342903 
19.33981442593822 -3.1000000000000005 
17.820185574061778 -4.6 
```
This part includes 4 vertices, numbering from 0 to 3

#### Edge
Edge defines connection between two nodes (as in non-directional graph). As each node defined in navmesh is a polygon, edge is constructed by 2 vertices and the connected two nodes. Similar as vertex, the edge is also number from 0. For example:
```
## format
# [Number]
# [Vertex0] [Vertex1] [Node0] [Node1]

2
2 3 0 22 
4 6 1 10 
```

**Note:**
1. Edge is non-directional, means `2 3 0 22` is equal to `2 3 22 0`, although it doesn't matter if navmesh includes them both.
2. Edge with same node number means nothing, because one node must be connected with itself.
3. To avoid ambiguity, a navmesh should avoid multiple definition for edges constructed on the same vertices.

#### Obstacle
Obstacle defines the edge that agent will never pass through. It should be always noted that an obstacle must be an edge of a polygon node. Contrary to edge, an obstacle defines un-connection between nodes, and thus an obstacle only belongs to one polygon. Each obstacle is numbered from 0. For example:
```
## format
# [Number]
# [Vertex0] [Vertex1] [Node] [Neighbor_obstacle]

1 
1 6 10 -1 
```

`[Neighbor_obstacle]` is usually set as -1.

#### Node
Node defines the accessible area for agents, which means, agent should be able to access all the coordinates within this node without any obstacle. Node is geometrically defined as a polygon. One node is constructed by `vertices`, `plane gradient`, `edges`, and `obstacles`, where `vertices`, `edges`, `obstalces` references the list number assigned above. Each node is number from 0. For example:

```
## format
# [Area_name]
# [Number]

Walkable
2

# [center_coordinates_X] [center_coordinates_Y]
# [vertices_number] [V0] [V1] ...
# [plane_gradient_A] [plane_gradient_B] [plane_gradient_C]
# [edges_number] [e0] [e1] ...
# [obstacles_number] [o0] [o1] ...

18.703371473730833 -3.9749893911585725 
4 0 1 2 3 
0 0 0 
2 0 1 
0
```

**Note:**
1. The vertices number must be listed as sequence, which means each vertex must be next to the geometrically neighbor vertices in the node polygon. In order to find whether a target point is within the node, a test will be conducted. In each traverse process of this test, 2 neighbor edges will be used. In `Menge::NavMeshPoly::containsPoint` method, the neighbor edges is constructed using consecutive vertices. 
2. The `plane_gradient` is related to the elevation feature of menge. It uses Ax + By + Cz = 0 to define the plane gradient.
3. center_coordinates is not necessarily the geometrical center of the polygon, but the center must be within the polygon area.


### Auto generate navmesh (building_navmesh)
Instead of generate polygon mesh for the whole area (Because there are a lot of area that agent will not access in the simulation, or for each state, agent will not be able to access everywhere), building_navmesh uses lanes with a certain width to generate the navmesh.

A lane is define by 2 lane vertices and 1 width. Referencing the navmesh requirements, there will be two types of node in auto generated navmesh: **intersection node** and **lane node**.

Intersection node is the node where >= 2 lanes intersects at the same lane vertex. Intersection node is working like a hub, a intersection node can only connected with lane nodes. **An intersection node has no obstacles!** While lane node is the polygon parallel following the lane direction with a vertain width. A lane node can only connected with intersection node. A lane node can be a dead end node if one of the lane vertices is not a intersection lane vertex.

The construction logic is:
1. Each 2 neighbor lane intersection at the same lane vertex will generate a polygon vertex because the lane width. Circulating 360 degress based on the intersection lane vertex will get all the polygon vertices for intersection node. 
2. Following 1, first generate all the intersection nodes, with polygon vertices and edges. 
3. Generating lane node based on the already generated polygon vertices and edges.
4. Cover the dead end lane node with 2 more polygon vertices. 
5. **Add obstalces for all lane node.** An obstalce is the edge that parallel with the lane, and the dead end edge.

There are few special cases:
1. 2 lanes intersects 1 lane vertex, circulating 360 degrees there are only 2 polygon vectices can be generated. 2 more polygon vertices are added on each lane nodes to construct the intersection node.
2. Intersection lanes are parallel, and sometimes the lane width is the same. Under this situation, the average lane width will be used, and the generated polygon vertex will be set along the normal direction of the lane direction with the average lane width. In this case, there might be a risk that the intersection node will invade the physical unaccessible area. As such, it is suggested to avoid huge lane width changes between connected lanes. 


## Navmesh generation logic


