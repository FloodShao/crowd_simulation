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

## menge setup files explaination
When generating the navmesh file from `office.menge_navmesh.building.yaml` 



