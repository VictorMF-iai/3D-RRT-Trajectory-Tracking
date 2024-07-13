import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d


class KeyFrame():
    def __init__ (self):
        self.pointcloud = None

    def from_file (self, filename):
        self.pointcloud = o3d.io.read_point_cloud(filename, print_progress=True)

    def from_points (self, points):
        self.pointcloud = o3d.geometry.PointCloud()
        self.pointcloud.points = o3d.utility.Vector3dVector(points)

    def save_pointcloud (self, output_filename):
        o3d.io.write_point_cloud(output_filename, self.pointcloud)

    def draw_cloud (self):
        o3d.visualization.draw_geometries([self.pointcloud],
                                          zoom=0.3412,
                                          front=[0.4257, -0.2125, -0.8795],
                                          lookat=[2.6172, 2.0475, 1.532],
                                          up=[-0.0694, -0.9768, 0.2024])

    def draw_cloud_plt (self):
        points = np.asarray(self.pointcloud.points)
        plt.figure()
        ax = plt.axes(projection="3d")

        # Creating plot
        ax.scatter3D(points[:, 0], points[:, 1], points[:, 2], color="blue")
        plt.title("LiDAR points 3D")
        # show plot
        plt.show()

    def downsample (self, voxel_size):
        self.pointcloud = self.pointcloud.voxel_down_sample(voxel_size=voxel_size)

    def transform (self, T):
        # pointcloud = self.pointcloud.uniform_down_sample(every_k_points=point_cloud_sampling)
        self.pointcloud = self.pointcloud.transform(T)

    def compute_traversability (self, Zupper=0.2, Zlower=-0.07):
        points = np.asarray(self.pointcloud.points)
        # filter
        # idx_traversable = points[:, 2] <= Zupper
        # idx_obstacle = points[:, 2] > Zupper
        idx_traversable = []
        idx_obstacle = []
        for i in range(len(points)):
            if Zlower < points[i, 2] <= Zupper:
                idx_traversable.append(True)
            else:
                idx_traversable.append(False)
            if points[i, 2] > Zupper:
                idx_obstacle.append(True)
            elif points[i, 2] <= Zlower:
                idx_obstacle.append(True)
            else:
                idx_obstacle.append(False)
        return points[idx_traversable, :], points[idx_obstacle, :]
