"""
Trajectory smoother
"""
from scipy.interpolate import splprep, splev
import numpy as np
# import matplotlib.pyplot as plt


class TrajectorySmoother():
    """
    A node in the tree, storing its id, parent_id and coordinates.
    """
    def __init__(self, s=2):
        self.s = s

    def smooth2D(self, trajectory):
        num_traj_points = len(trajectory)
        x_sample = trajectory[:, 0]
        y_sample = trajectory[:, 1]
        tck, u = splprep([x_sample, y_sample], s=self.s)
        # x_knots, y_knots, z_knots = splev(tck[0], tck)
        u_fine = np.linspace(0, 1, num_traj_points)
        x_fine, y_fine = splev(u_fine, tck)
        return np.column_stack((x_fine, y_fine))

    def smooth3D(self, trajectory):
        num_traj_points = len(trajectory)
        x_sample = trajectory[:, 0]
        y_sample = trajectory[:, 1]
        z_sample = trajectory[:, 2]
        tck, u = splprep([x_sample, y_sample, z_sample], s=self.s)
        # x_knots, y_knots, z_knots = splev(tck[0], tck)
        u_fine = np.linspace(0, 1, num_traj_points)
        x_fine, y_fine, z_fine = splev(u_fine, tck)
        return np.column_stack((x_fine, y_fine, z_fine))