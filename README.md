# GaTORS: A Game-Theoretic Tool for Optimal Robot Selection and Design in Surface Coverage Applications

[![github.io](https://img.shields.io/badge/github.io-MainPage-black.svg)](https://utnuclearroboticspublic.github.io/gators/)

![](images/i3.png)

## About
As the number of commercially-available robots
increases, users face the challenge of evaluating many options
to identify the optimal system for their needs. This market
saturation also compels providers to ensure new systems are
competitive with or superior to existing robots to increase
economic viability. The need for evaluation extends to multi-
robot teams collaborating toward shared objectives, where
understanding individual contributions to overall team per-
formance is complex but necessary. One specific application
domain for robot platform selection in industry is autonomous
surface coverage, which includes tasks such as painting, clean-
ing, and surveying in industrial facilities. To assist in the
design and selection of robotic systems for surface coverage
applications, we introduce GaTORS, a novel tool that frames
the surface coverage task allocation process as a collaborative
general-sum discrete-time game. By parameterizing robots with
a set of common constraints, this tool enables performance
evaluation of existing and potential future robotic systems.
GaTORS is evaluated in a case study of surface coverage for
corrosion mitigation in an industrial refinery, with experiments
demonstrating its utility in selecting existing robotic platforms
best suited to complete the specific coverage task. These
experiments also highlight GaTORS’ potential to inform the
design of new systems that can efficiently accomplish assigned
tasks within practical time and cost constraints. Due to its
flexibility, GaTORS can be easily adapted to provide similar
insights for other types of robots in different environments and
surface coverage applications.

## Contents
**gators** is a ROS2-based package. However, ROS is only neccessary for visualization; all game mechanics and execution are pure C++ that can be used independently.  

## Requirements, Dependencies, and Building
GaTORS is built and tested on a system running ROS2 Humble on Ubuntu 22.04. However, the project is written entirely in C++ and wrapped in a thin ROS wrapper for visualization purposes, so it can easily be compiled and run independently of ROS, just without the shown visualizations. 

1. Create a Catkin workspace:
```
mkdir -p colcon_ws/src && cd colcon_ws
```
2. Clone the contents of this repository:
```
git clone git@github.com:UTNuclearRoboticsPublic/gators.git src/
```
3. Install all package dependencies:
```
rosdep update
```
```
rosdep install --from-paths src --ignore-src -r -y
```
4. Download meshes and clouds used for demonstration by following the instructions in ```models/info.txt```.

5. Build and source the workspace:
```
colcon build
```
```
source install/local_setup.bash
```

## Running an Example with GaTORS
Parameters for numbers of players, game and repair board files and discretizations, and Monte Carlo tree search parameters can be changed in ```config/params.yaml```. Make sure the parameter ```/gators/game_player.ros__parameters.gators.pkg_path``` is updated to reflect the package's location on your machine. If this is changed, GaTORS must be rebuilt to copy the updated parameter file to the workspace ```share``` directory:
```
colcon build
```

Once parameters are set, the example simulation can be run with:

```
ros2 launch gators load_game.launch.py
```

in one terminal, followed by 
```
ros2 service call /gators/game_player/play_game std_srvs/srv/Trigger
```
in a separate terminal to simulate the full game.


[](https://github.com/steven-swanbeck/game_theoretic_painting/assets/99771915/644db458-211a-4ba5-9886-ede0299eddec)

![](images/game_simulation.mp4)

## Customizing to Fit Your Applications
GaTORS was orignally designed for providing robot design and selection insights for surface coverage problems. As such, the parameterization used for the game and robots is tailored to coverage-relevant parameters. GaTORS' ```Robot``` class is located in ```src/agents.cpp```, and parameters for the ```Drone```, ```Quadruped```, and ```Gantry``` robots used in the original paper can be adjusted inside ```include/gators/agents.hpp```. 

To support new robot types or extend GaTORS to applications significantly different from its demonstration use, users should take the following steps:
1. Edit the ```Robot``` class in ```src/agents.cpp``` to add any additional constraints that may be relevant for your application.

2. Create any custom ```Robot``` classes in ```include/gators/agents.hpp``` to describe the systems to be used. 

3. Edit or create a new party instantiation function using ```instantiatePlayers()``` in ```src/agents.cpp``` as a template. 

4. Model any movement limitations for new systems with a function in ```src/board.cpp``` using the ```assignGantryEdges()``` function as a template. 

5. If visualization is desired, add a mesh file for your custom environment or robot to the ```models/meshes/``` folder.

## Additional Info

Currently supported robot types are drones, quadrupeds, and gantries.

![](images/i4.png)

The game board is built using input point clouds that represent the map of the environment and the material within it that must be repaired.

![](images/i5.png)

The robots play on this board until all material in the environment has been repaired.

![](images/i1.png)
