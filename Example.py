from robots.ouster import Ouster
from robots.simulation import Simulation
from Husky import HuskyRobot
from robots.objects import CoppeliaObject
from overpass_routes import getTrajNodes
import Traj_mod as tr
from artelib.homogeneousmatrix import HomogeneousMatrix
from keyframe.keyframe import KeyFrame
from artelib.rotationmatrix import RotationMatrix
import numpy as np
from run_rrt_planner_pc import find_path_connect_to_goalPC


def start ():
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = HuskyRobot(simulation=simulation)
    robot.start(base_name='/HUSKY')
    # Get robot parameters
    robot.getParams()
    # Simulate a LiDAR
    lidar = Ouster(simulation=simulation)
    lidar.start(name='/OS1')
    # Simulate center of Husky robot
    base = CoppeliaObject(simulation=simulation)
    base.start(name='/CentroHusky')
    # Coordinate 0
    coord0 = [38.275401, -0.686178]
    # Coordinate of our origin point
    origin_point = [38.275401, -0.686178]
    # Coordinate of our destination point
    destination_point = [38.2763131, -0.6866021]
    # Getting the nodes of the path found
    data = getTrajNodes(origin_point, destination_point)
    zh = 0.243  # Height of the center of the robot
    traj = []
    # We convert lat/long coordinates to UTM
    for i in range(len(data)):
        x, y = tr.convertCoordinates(coord0, data[i])
        z = zh
        traj.append([x, y, z])
    # Linear interpolation of the nodes
    newtraj = tr.interpTraj(traj, 3)
    # Spline interpolation on the curves to smoothen the path
    goal = tr.SplinTraj(newtraj, traj)
    voxel_size = 0.3

    for i in range(len(goal)):
        # Convert nodes to robot's local coordinates
        obj = tr.CoordsToLocal(base, goal[i])
        # Get LiDar data
        data_lidar = get_lidar_data(lidar, base, voxel_size)
        # Find RRT path
        li_traj, valid, _, _ = find_path_connect_to_goalPC(data_lidar, obj, base.get_position(), voxel_size)
        # Convert path to Global coordinates
        if not valid:
            continue
        glb_traj = []
        for punto in li_traj:
            glb_traj.append(tr.CoordsToGlobal(base, punto))

        # Follow path found
        robot.goto_objective(puntos=glb_traj, base=base, velocity=0.3, skip=True)
    simulation.stop()


def get_lidar_data (lidar, base, vs):
    # get lidar data

    data = lidar.get_laser_data()

    print('Received Laser Data')
    try:
        print(data.shape)
        print(data.dtype)
    except:
        print('Unknown data type')
    pcl = KeyFrame()
    pcl.from_points(data)
    pcl.downsample(voxel_size=vs)

    # Convert CoppeliaSim LiDar orientation to desired orientation
    T_base = base.get_transform()
    T2 = HomogeneousMatrix([0, 0, 0], T_base.euler()[0])
    mrot = RotationMatrix([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    meuler = mrot.euler()[0]
    T3 = HomogeneousMatrix([0, 0, 0.925], meuler)
    tx = T2 * T3
    pcl.transform(tx.toarray())

    # pcl.draw_cloud_plt()
    return np.asarray(pcl.pointcloud.points)


if __name__ == "__main__":
    start()
