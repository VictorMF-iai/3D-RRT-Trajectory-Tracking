# Description

The main objective is to develop a trajectory tracking system from GPS coordinates for a mobile robot using LiDAR point
clouds. This system will allow the robot to navigate autonomously in complex environments, ensuring a high accuracy in
obstacle detection and route planning.

* Made for differential wheeled mobile robot. The HUSKY robot in particular.
* Using RRT-based algorithm on LiDAR pointclouds for local path planning.
* Tested and simualated in CoppeliaSim.

Created by Víctor Márquez victormarferiai@gmail.com

![Husky](/readme_sources/husky.png)

# Requirements

```
-matplotlib
-numpy
-yaml
-utm
-scipy
-pyARTE (https://github.com/4rtur1t0/pyARTE)
```

# Directory structure

* **/path_planning**: Contains the script that gives us the global path form the origin and destination coordinates. For
  that, we have exported the map data from https://overpass-turbo.eu/ and using Dijkstra algorithm we found the shortest
  path.
* **Traj_mod**: Contains varoius functions that modifiy the global trajectory found making it more suitable for the
  local planner.
* **/RRTPlanner**: This folder contains a slight modification of a repository that implements the RRT algorithm for path
  planning for holonomic robots. You can find the original repository here: https://github.com/4rtur1t0/RRTPlanner. In
  our case we have modified it to make it suitable for our application.
* **Husky.py**: This script contains the class Husky that includes its motion control and path-following controller.
* **Example.py**: Contains an example script.
* **/scenes**: Contains CoppeliaSim scenes.

# Basic RRT algorithm

![RRT](/readme_sources/RRT.GIF)

# Tests

|                                                                       | VIDEO                                  | RRT DECISIONS                          |
|-----------------------------------------------------------------------|----------------------------------------|----------------------------------------|
| Test 1: Fully-delimited narrow path                                   | ![Vid1](/readme_sources/nav_vid_1.gif) | ![RRT1](/readme_sources/nav_rrt_1.gif) |
| Test 2: Fully-delimited path with obstacles                           | ![Vid2](/readme_sources/nav_vid_2.gif) | ![RRT2](/readme_sources/nav_rrt_2.gif) |
| Test 3: Semi-delimited path with crowds of people                     | ![Vid2](/readme_sources/nav_vid_3.gif) | ![RRT2](/readme_sources/nav_rrt_3.gif) |
| Test 4: Path on the pavemient with big height difference on the sides | ![Vid2](/readme_sources/nav_vid_4.gif) | ![RRT2](/readme_sources/nav_rrt_4.gif) |
| Test 5: Irregular path with obstacles                                 | ![Vid2](/readme_sources/nav_vid_5.gif) | ![RRT2](/readme_sources/nav_rrt_5.gif) |


