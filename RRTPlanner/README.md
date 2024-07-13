# RRTPlanner
A simple, yet intuitive, implementation of the RRT algorithm for path planning for holonomic robots.

Two main algorithms are implemented:
- 2D examples with:
    - RRT: The basic algorithm.
    - RRT-connect: a variant using 2 trees wich try to connect directly to each other in each iteration of the algorithm.
    - RRT-connect-to-goal: a variation of RRT-connect with a single tree that tries to connect to the final goal at each iteration of the algorithm.
 
    An example usage of the three can be found in run_rrt_planner.py. The resulting trajectory of each algorithm can be smoothed. 

- 3D examples working on LiDAR points:
  This is the most interesting algorithm. Points from the LiDAR are classified either as traversable or as non-traversable (obstacles).
  The goal is to navigate from qstart to qgoal in this environment while:
  a) Traversing always over traversable LiDAR points (at least one LiDAR point).
  b) With no obstacle LiDAR points in a robot radius.
    - RRT PC: The basic algorithm running on point clouds.
    - RRT-connect PC: a variant using 2 trees wich try to connect directly to each other in each iteration of the algorithm.
    - RRT-connect-to-goal PC: a variation of RRT-connect with a single tree that tries to connect to the final goal at each iteration of the algorithm.

    An example usage of the three can be found in run_rrt_planner_pc.py. The resulting trajectory of each algorithm can be smoothed.



# Requirements and installation
Below, an installation on a virtual environment is presented

```
>> sudo apt install virtualenv
>> sudo apt install python3-dev
>> sudo apt install git
>> git clone https://github.com/4rtur1t0/RRTPlanner
>> virtualenv venv
>> venv/bin/pip install -r RRTPlanner/requirements.txt
```

Please configure venv/bin/python as your python interpreter.



# References:
- RRT-connect: An efficient approach to single-query path planning. J. J. Kuffner and S. M. LaValle. In Proceedings IEEE International Conference on Robotics and Automation, pages 995-1001, 2000. [pdf].

- Rapidly-exploring random trees: A new tool for path planning. S. M. LaValle. TR 98-11, Computer Science Dept., Iowa State University, October 1998, [pdf].