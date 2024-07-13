#!/usr/bin/env python
# encoding: utf-8
"""
Find a path in a 2D environment using RRT.

Several variants of the RRT algorithm are here tested.

a) RRT basic. The standard RRT algorithm.
    Two slight modifications are performed over the stantard RRT. The algorithm returns, always, as a solution:
        a.1) The path to the goal. This is the first path found by the algorithm.
             Of course, it may not be the shortest path.
        a.2) If the path is nof found in max_nodes, iterations. The algorithm looks for the closest node to
             the goal. Then, the path that connects the start node and the nearest neighbour node is
             returned.
    In the RRTPlannerBasic class, a solution is always found by the planner, either  by reaching the goal or
    by finding the closest node to the goal. This, of course, is a desired behaviour, since the robot
    has always a movement plan to carry out.

b) RRT connect. In this case the algorithm repeats an extend operation trying to connect
   the two trees after a random_config is added to the tree.
c) RRT connect to the goal. At each iteration, the algorith tries to connect a single tree to the goal.

A holonomic robot is considered.
The robot is considered a point. The robot is inside an obstacle if the point lies on any of the specified circles.

@Authors: Arturo Gil
@Time: April 2024
"""
from rrtplanner.rrtplanner import RRTPlanner
import matplotlib.pyplot as plt
from rrtplanner.trajectorysmoother import TrajectorySmoother


def find_path_RRT_basic():
    smoother = TrajectorySmoother(s=3)
    # define the dimensions +-X, +-Y of the map
    dimensions = [20, 20]
    # start and end position
    start = [-8, -8]
    goal = [15, 15]
    # define a list of obstacles as (x, y, Radius)
    obstacles = [[0, 0, 3],
                 [0, 15, 3],
                 [5, 5, 3],
                 [15, -10, 3],
                 [0, -3, 2]]
    # a solution is always found by the planner, either  by reaching the goal or finding the node in the tree
    # which is closest to the goal
    planner = RRTPlanner(start=start,
                         goal=goal,
                         dimensions=dimensions,
                         obstacles=obstacles,
                         epsilon=1,
                         max_nodes=2000)
    tree = planner.build_rrt_basic()
    planner.print_info()
    # retrieve found path path from tree
    path_found = planner.get_solution_path(tree)
    print('Path found: ', path_found)
    # optionally, find a smooth path
    path_smooth = smoother.smooth2D(path_found)
    print('Smoothed path: ', path_found)
    # plot the tree and solution
    tree.plot()
    tree.plot_path(path=path_found, color='cyan')
    tree.plot_path(path=path_smooth, color='green')
    # plot obstacles, start and goal
    planner.plot()
    # update plots
    plt.show(block=True)
    print('FINISHED')


def find_path_RRT_connect():
    smoother = TrajectorySmoother(s=3)
    # define the dimensions +-X, +-Y of the map
    dimensions = [30, 30]
    # start and end position
    start = [-8, -8]
    goal = [15, 15]
    # define a list of obstacles as (x, y, Radius)
    obstacles = [[0, 0, 6],
                 [0, 15, 5],
                 [5, 5, 6],
                 [15, -10, 6],
                 [0, -3, 2]]
    # a solution is always found by the planner, either  by reaching the goal or finding the node in the tree
    # which is closest to the goal
    planner = RRTPlanner(start=start,
                         goal=goal,
                         dimensions=dimensions,
                         obstacles=obstacles,
                         epsilon=1,
                         max_nodes=2000)
    # returns two trees that may be connected
    treeA, treeB = planner.build_rrt_connect()
    planner.print_info()
    # retrieve found paths from trees
    path_found = planner.get_solution_path_from_two_trees(treeA, treeB)
    print('Optimal path: ', path_found)

    # optionally, find a smooth path
    path_smooth = smoother.smooth2D(path_found)
    print('Smoothed path: ', path_smooth)

    # plot the tree and solution
    treeA.plot()
    treeB.plot()
    treeA.plot_path(path=path_found, color='cyan')
    treeA.plot_path(path=path_smooth, color='green')
    # plot obstacles, start and goal
    planner.plot()

    # update plots
    plt.show(block=True)
    print('FINISHED')


def find_path_RRT_connect_to_goal():
    smoother = TrajectorySmoother(s=3)
    # define the dimensions +-X, +-Y of the map
    dimensions = [30, 30]
    # start and end position
    start = [-8, -8]
    goal = [15, 15]
    # define a list of obstacles as (x, y, Radius)
    obstacles = [[0, 0, 3],
                 [0, 15, 5],
                 [5, 5, 3],
                 [15, -10, 5],
                 [0, -3, 2]]
    # a solution is always found by the planner, either  by reaching the goal or finding the node in the tree
    # which is closest to the goal
    planner = RRTPlanner(start=start,
                         goal=goal,
                         dimensions=dimensions,
                         obstacles=obstacles,
                         epsilon=1,
                         max_nodes=2000)
    # returns two trees that may be connected
    tree = planner.build_rrt_connect_to_goal()
    planner.print_info()
    # retrieve found paths from trees
    path_found = planner.get_solution_path(tree)
    print('Optimal path: ', path_found)

    # optionally, find a smooth path
    path_smooth = smoother.smooth2D(path_found)
    print('Smoothed path: ', path_smooth)

    # plot the tree and solution
    tree.plot()
    tree.plot_path(path=path_found, color='cyan')
    tree.plot_path(path=path_smooth, color='green')
    # plot obstacles, start and goal
    planner.plot()

    # update plots
    plt.show(block=True)
    print('FINISHED')


if __name__ == "__main__":
    # please uncomment as necessary
    # three RRT variants are here tested
    find_path_RRT_basic()
    find_path_RRT_connect()
    find_path_RRT_connect_to_goal()

