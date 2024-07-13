"""
    The tree class.
"""
import numpy as np
import matplotlib.pyplot as plt
from rrtplanner.node import Node


class Tree():
    def __init__(self, qinit):
        # stores the nodes of the tree
        self.tree = []
        # add root node to tree: start the root of the tree
        self.add_vertex(parent_id=0, coordinates=qinit)

    def add_vertex(self, parent_id, coordinates, goal_reached=False):
        # new id depending on the number of nodes
        id = len(self.tree)
        node = Node(id=id, parent_id=parent_id, coordinates=coordinates, goal_reached=goal_reached)
        self.tree.append(node)

    def get_coordinates(self):
        """
        Gets the coordinates of all nodes in the tree.
        """
        coords = []
        # build a list to compute distance fast with numpy?
        for i in range(len(self.tree)):
            coords.append(self.tree[i].coordinates)
        coords = np.array(coords)
        return coords

    def nearest_neighbour(self, q):
        coords = self.get_coordinates()
        dists = np.linalg.norm(coords - q, axis=1)
        index_nn = np.argmin(dists)
        return self.tree[index_nn]

    def find_node_reached(self):
        """
        Finds the node that reached the target. Iterating through the list of nodes in reverse order is always faster.
        :return:
        """
        for i in reversed(range(len(self.tree))):
            if self.tree[i].goal_reached:
                return self.tree[i]
        return None

    def mark_node_as_reached(self, index):
        """
        Mark the node as reached. Used in rrt connect
        :return:
        """
        self.tree[index].goal_reached = True
        return

    def back_trace_path_from_node(self, node):
        path = []
        while True:
            path.append(node.coordinates)
            # this happens only at the root node with id==parent_id==0
            if node.id == node.parent_id:
                break
            # backtrace the node
            node = self.tree[node.parent_id]
        return path

    def print_info(self):
        print(30*'*')
        for i in range(len(self.tree)):
            print('ID, reached: ', self.tree[i].id, self.tree[i].goal_reached)
        print(30 * '*')

    def plot(self):
        # plot the edges of the tree
        for i in range(len(self.tree)):
            coords = self.tree[i].coordinates
            parent_id = self.tree[i].parent_id
            coords_parent = self.tree[parent_id].coordinates
            c = np.vstack((coords, coords_parent))
            plt.plot(c[:, 0], c[:, 1], color='magenta')
        # plot vertices (node coordinates) as scatter
        coords = self.get_coordinates()
        plt.scatter(coords[:, 0], coords[:, 1], color='magenta')

    def plot_path(self, path, color='cyan'):
        solution_path = np.array(path)
        K = solution_path.shape[0]
        for i in range(K-1):
            coords = solution_path[i, :]
            coordsn = solution_path[i+1, :]
            c = np.vstack((coords, coordsn))
            plt.plot(c[:, 0], c[:, 1], color=color)
        plt.scatter(path[:, 0], path[:, 1], color=color)
