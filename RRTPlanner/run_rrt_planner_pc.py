#!/usr/bin/env python
# encoding: utf-8
"""
Find a path on a point cloud.
Each point in the pointcloud is classified as traversable or non-traversable.
We are looking for a 3D trajectory using RRT, with the following characteristics:
    - The robot (sphere), should have traversable points, ensuring that there exists a surface to navigate on.
    - In addition the robot (sphere), should not have non-traversable points, considered as obstacles.
    - An RRT Planning with multiple goals considered.
    - A holonomic robot is considered.

- Under development:
    - Find a smooth trajectory.
    - Integrate RRT CONNECT.
    - Integrate max delta(phi) between nodes in the tree.
    - Integrate max delta(Z) between the points.

    - Probabilistic roadmap.

Two slight modifications are performed over the stantard RRT:
    a) The algorithm looks for a path to any of the goals specified.
    b) The algorithm stops when a path is found for any of the goals.
    c) The algorithm returns, always, as a solution:
        b.1) The paths that connects the start and one of the goals.
        b.2) If no path was found, the algorithm finds the closest node to any of the goals and returns the path that
        connects the nearest neighbour node with the start node.

In the RRTPlannerMG a solution is always found by the planner, either  by reaching any of the goals or
by finding the closest node to any of the goals. This, of course, is a desired behaviour, since the robot
has always a movement plan to carry out.


@Authors: Arturo Gil
@Time: April 2024
"""
import numpy as np
import matplotlib.pyplot as plt
from keyframe.keyframe import KeyFrame
from rrtplanner.rrtplannerPC import RRTPlannerPC
from rrtplanner.trajectorysmoother import TrajectorySmoother


def find_path_RRT_basic_point_cloud ():
    smoother = TrajectorySmoother(s=5)
    # transformation from the robot center (at the ground) to the LIDAR.
    T = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0.8],
                  [0, 0, 0, 1]])
    # simulation: must subscribe to lidar and compute traversability
    pcl = KeyFrame()
    pcl.from_file('dataset/robot0/lidar/001.pcd')
    pcl.downsample(voxel_size=0.2)
    pcl.transform(T)
    # pointcloud.draw_cloud()
    # a simple classification in traversable/non-traversable if any points surpasses Z
    points_traversable, points_obstacles = pcl.compute_traversability(Z=0.1)

    # start and goals
    start = [0, 0, 0]
    # goals should be projected to the local reference system from the global UTM coordinates
    goal = [12, 2, 0]
    # Create Multiple Goal RRT planner
    planner = RRTPlannerPC(start=start,
                           goal=goal,
                           pc_traversable=points_traversable,
                           pc_obstacles=points_obstacles,
                           robot_radius=0.7,
                           epsilon=0.5,
                           max_nodes=3500)

    tree = planner.build_rrt_basic()
    planner.print_info()
    # retrieve found path path from tree
    path_found = planner.get_solution_path(tree)
    print('Optimal path: ', path_found)

    # optionally, find a smooth path
    path_smooth = smoother.smooth3D(path_found)
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


def find_path_RRT_connect_point_cloud ():
    smoother = TrajectorySmoother(s=5)
    # transformation from the robot center (at the ground) to the LIDAR.
    T = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0.8],
                  [0, 0, 0, 1]])
    # simulation: must subscribe to lidar and compute traversability
    pcl = KeyFrame()
    pcl.from_file('dataset/robot0/lidar/001.pcd')
    pcl.downsample(voxel_size=0.2)
    pcl.transform(T)
    # pointcloud.draw_cloud()
    # a simple classification in traversable/non-traversable if any points surpasses Z
    points_traversable, points_obstacles = pcl.compute_traversability(Z=0.1)

    # start and goals
    start = [0, 0, 0]
    # goals should be projected to the local reference system from the global UTM coordinates
    goal = [15, 0, 0]
    # goal = [-8, -8.5, 0]
    # Create Multiple Goal RRT planner
    planner = RRTPlannerPC(start=start,
                           goal=goal,
                           pc_traversable=points_traversable,
                           pc_obstacles=points_obstacles,
                           robot_radius=0.7,
                           epsilon=0.5,
                           max_nodes=3500)

    # returns two trees that may be connected
    treeA, treeB = planner.build_rrt_connect()
    planner.print_info()
    # retrieve found paths from trees
    path_found = planner.get_solution_path_from_two_trees(treeA, treeB)
    print('Optimal path: ', path_found)

    # optionally, find a smooth path
    path_smooth = smoother.smooth3D(path_found)
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


def find_path_RRT_connect_to_goal_point_cloud ():
    smoother = TrajectorySmoother(s=5)
    # transformation from the robot center (at the ground) to the LIDAR.
    T = np.array([[0, 0, 1, 0],
                  [1, 0, 0, 0],
                  [0, 1, 0, 0.925],
                  [0, 0, 0, 1]])
    # simulation: must subscribe to lidar and compute traversability
    pcl = KeyFrame()
    pcl.from_file('C:/Users/User/pruebasTFG/Pointcloud/pruebaCoppeliaRecto.pcd')
    # pcl.from_points(np.asarray(pcl.pointcloud.points))
    pcl.downsample(voxel_size=0.5)

    # pcl.draw_cloud()
    # pcl.draw_cloud_plt()

    pcl.transform(T)

    # pcl.draw_cloud_plt()

    # a simple classification in traversable/non-traversable if any points surpasses Z
    points_traversable, points_obstacles = pcl.compute_traversability(Z=0.2)
    # start and goals
    start = [0, 0, 0]
    # goals should be projected to the local reference system from the global UTM coordinates
    goal = [0.0, 8, 0]
    # goal = [-2, 10, 0]
    # Create Multiple Goal RRT planner
    try:
        planner = RRTPlannerPC(start=start,
                               goal=goal,
                               pc_traversable=points_traversable,
                               pc_obstacles=points_obstacles,
                               epsilon=0.7,
                               robot_radius=0.7,
                               max_nodes=3500)
    except:
        print("salgo del punto")
    print("continua")
    # returns two trees that may be connected
    tree = planner.build_rrt_connect_to_goal()
    planner.print_info()
    # retrieve found path path from tree
    path_found = planner.get_solution_path(tree)
    print('Optimal path: ', path_found)

    # optionally, find a smooth path
    path_smooth = smoother.smooth3D(path_found)
    print('Smoothed path: ', path_smooth)
    goal_path = np.array([[0, 3, 0], [0, 6, 0], [0, 9, 0], [-0.2, 12, 0], [-0.4, 15, 0]])
    # plot the tree and solution
    tree.plot()
    tree.plot_path(path=path_found, color='cyan')
    tree.plot_path(path=path_smooth, color='green')
    tree.plot_path(path=goal_path, color='purple')
    # plot obstacles, start and goal
    planner.plot()

    # update plots
    plt.show(block=True)
    print('FINISHED')


def find_path_connect_to_goalPC (data, goal, start, vs):
    smoother = TrajectorySmoother(s=5)

    pcl = KeyFrame()
    pcl.from_points(data)
    # pcl.from_points(np.asarray(pcl.pointcloud.points))
    pcl.downsample(voxel_size=vs)

    # pcl.draw_cloud()

    # pcl.draw_cloud_plt()

    # a simple classification in traversable/non-traversable if any points surpasses Z
    points_traversable, points_obstacles = pcl.compute_traversability(Zupper=0.15,Zlower=-0.07)

    start = [0, 0, 0]
    try:
        planner = RRTPlannerPC(start=start,
                               goal=goal,
                               pc_traversable=points_traversable,
                               pc_obstacles=points_obstacles,
                               epsilon=0.7,
                               robot_radius=0.7,
                               max_nodes=5000)
    except:
        return [], False, 0, False
    # returns two trees that may be connected
    tree = planner.build_rrt_connect_to_goal()
    planner.print_info()
    # retrieve found path path from tree
    path_found = planner.get_solution_path(tree)
    print('Optimal path: ', path_found)
    goal_path = np.array([goal])
    # plot the tree and solution
    tree.plot()
    tree.plot_path(path=path_found, color='cyan')
    tree.plot_path(path=goal_path, color='purple')
    # optionally, find a smooth path
    try:
        path_smooth = smoother.smooth3D(path_found)
        print('Smoothed path: ', path_smooth)
        tree.plot_path(path=path_smooth, color='green')
    except:
        path_smooth = path_found
    # plot obstacles, start and goal
    planner.plot()

    # update plots
    plt.show(block=True)
    print('FINISHED')
    np.append(path_smooth, goal)
    return path_smooth, True, planner.iterations_performed, planner.goal_reached


if __name__ == "__main__":
    # find_path_RRT_basic_point_cloud()
    # find_path_RRT_connect_point_cloud()
    find_path_RRT_connect_to_goal_point_cloud()
