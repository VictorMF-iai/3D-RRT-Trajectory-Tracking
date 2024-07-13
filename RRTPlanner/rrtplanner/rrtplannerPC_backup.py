"""
    The main class RRTPlannerPC and the node class.

    Planning on a point cloud.

    The point cloud points are either classified as traversable or non-traversable.

    The code is very similar to the RRTPlannerBasic. Both codes are maintained separately for Educational purposes.
    This constitutes a demo of a 2D planning of a holonomic robot with circle-like obstacles.
    A set of circle-like obstacles are considered.

    In this case, compared to RRTPlannerBasic, RRTPlannerMG considers multiple goals. The algorithm stops whenever any
    of the goals is reached.

    Please, read:
    - RRT-connect: An efficient approach to single-query path planning. J. J. Kuffner and S. M. LaValle. In Proceedings IEEE International Conference on Robotics and Automation, pages 995-1001, 2000. [pdf].
    - Rapidly-exploring random trees: A new tool for path planning. S. M. LaValle. TR 98-11, Computer Science Dept., Iowa State University, October 1998, [pdf].

    The minor variation with respect to the basic algorithm is that, after max_nodes iterations, the
    algorithm always returns a solution:
        a) If any the goals is reached, the path that connects the start and goal is returned.
        b) If no goal reached, the node that is found closest to any of the goals goal is found.
           Next, the path that connects the start and the closest node is returned.

    This behaviour is not optimal, however, it yields, sometimes, necessary to have always
    a local path to follow in the context of mobile robotics. It yields probable, that, in the next
    observations, a valid path is found that allows the robot to reach the goal.
"""
import numpy as np
import matplotlib.pyplot as plt


class RRTPlannerPC():
    """
    A Basic RRT Planner in a 2D space for holonomic robots.
    RRTPlannerMG considers multiple-goals, instead of just one as the basic RRT.
    """
    def __init__(self, start, goals, pc_traversable, pc_obstacle, epsilon, max_nodes):
        self.goals = np.array(goals)
        self.start = np.array(start)
        self.max_nodes = max_nodes
        self.epsilon = epsilon
        # a set of 3D traversable points
        self.pc_traversable = np.array(pc_traversable)
        # a set of 3D points considered non-traversable
        self.pc_obstacles = np.array(pc_obstacle)
        # stores the nodes of the tree
        self.tree = []
        # stores info on the result of the algorithm
        self.goal_reached = False
        self.iterations_performed = 0
        # Robot dimensions, here approximated as a sphere centered on each traversable point
        self.robot_radius = 0.7

        # initial seeds
        self.initial_radius = 2.0
        self.initial_deltaZ = 0.2

        #if self.in_obstacle(self.start):
        #    raise Exception('The start coordinates are inside the obstacle space')

        # The goals may be in the obstacle space
        # for i in range(self.goals.shape[0]):
        #     if self.in_obstacle(self.goals[i]):
        #         raise Exception('One of the goals is inside the obstacle space')

        # add the root node of the tree
        self.add_vertex(parent_id=0, coordinates=start)
        # this is a typical special case when working with LiDARs, the nearest points to the robot
        # are at a specified distance. We are adding points that:
        # are within max_dist radius.
        # with DeltaZ<0.2 m
        # we obtain a random choice of the result
        points = self.nearest_traversable_points(q=self.start, max_dist=self.initial_radius,
                                                 deltaZ=self.initial_deltaZ, Nmax=20)
        # yes, add, the points to the root node
        for i in range(points.shape[0]):
            self.add_vertex(parent_id=self.tree[0].id, coordinates=points[i, :])

    def build_rrt(self):
        for k in range(self.max_nodes):
            print('Iteration: ', k)
            qrand = self.random_config()
            reached = self.extend(qrand)
            if reached:
                self.goal_reached = True
                self.iterations_performed = k
                # self.plot_tree2d()
                break

    def add_vertex(self, parent_id, coordinates, reached=False):
        # new id depending on the number of nodes
        id = len(self.tree)
        node = Node(id=id, parent_id=parent_id, coordinates=coordinates, reached=reached)
        self.tree.append(node)

    def random_config(self):
        """
        This generates a new random configuration.
        In this particular problem, a random sample is selected from the list of traversable points.
        :return:
        """
        N = self.pc_traversable.shape[0]
        i = np.random.choice(N, 1)[0]
        return self.pc_traversable[i, :]

    def extend(self, qrand):
        """
        The extend operation of the tree.
        This is a main variation if compared to the basic RRT algorithms. The extension operation is summarized
        as follows:

        a) In this case, qrand is sampled from the set of points that form the pointcloud (and not generated randomly
        in a given workspace)
        b) Next, the closest node to the tree is found. This node is qnear.
        c) Next, a new qi is created by a linear propagation from qnear in the direction of qrand
                qi = qnear + u*epsilon
        d) Next, the closest point in the pointcloud to qi is found and consitutes the extension result.
        :param qrand:
        :return:
        """
        # given qrand, find the closest node of the tree
        inear, qnear = self.nearest_neighbour_in_tree(qrand)
        # this propagates a point from qnear to qrand at a distance epsilon
        qnew = self.new_config(qnear, qrand)
        # now, important, select a real point in the traversable space.
        inew, qnew = self.nearest_neighbour(self.pc_traversable, qnew)
        # now, check if qnew is next to an obstacle
        if not self.in_obstacle(qnew):
            # if not in collision, add node to the tree and check if we have reached the goal
            reached = self.reached_goal(qnew)
            self.add_vertex(parent_id=inear, coordinates=qnew, reached=reached)
            if reached:
                return True
        return False

    def new_config(self, qnear, qrand):
        """
        Computes a new configuration between qnear and qrand that is placed at an epsilon distance from qnear
        :param qnear: a node in the tree
        :param qrand: a random configuration
        :return:
        """
        ds = qrand - qnear
        n = np.linalg.norm(ds)
        if n > 0:
            du = ds/n
            qnew = qnear + self.epsilon * du
            return qnew
        return qnear

    def in_obstacle(self, q):
        """
        Returns True if q is in the obstacle space, false elsewise.
                Here, checking whether there are points in pc_obstacle within a radius R of the robot
        :param q:
        :return:
        """
        index, coords = self.nearest_neighbour(self.pc_obstacles, q)
        d = np.linalg.norm(q-coords)
        if d < self.robot_radius:
            return True
        return False

    def nearest_traversable_points(self, q, max_dist, deltaZ, Nmax):
        """
        Returns a set of Nmax points in the traversable set within max_dist distance.
        The points are sampled so that the max number is K.
        :param qrand:
        :return:
        """
        dists = np.linalg.norm(self.pc_traversable - q, axis=1)
        # plt.scatter(range(len(dists)), dists)
        index_within = dists <= max_dist
        # get the points
        points_within = self.pc_traversable[index_within, :]
        # get the points with DeltaZ < 0.2
        index_within = np.abs(points_within[:, 2]) < np.abs(deltaZ)
        points_within = points_within[index_within, :]
        if points_within.shape[0] > Nmax:
            indices_sampled = np.random.choice(points_within.shape[0], size=Nmax, replace=False)
            return points_within[indices_sampled, :]
        return points_within

    def nearest_neighbour_in_tree(self, qrand):
        """
        Returns the index in the tree that is nearest to qrand.
        Also returns the coordinates of the nearest index
        :param qrand:
        :return:
        """
        coords = []
        # build a list to compute distance fast with numpy?
        for i in range(len(self.tree)):
            coords.append(self.tree[i].coordinates)
        coords = np.array(coords)
        dists = np.linalg.norm(coords - qrand, axis=1)
        index_nn = np.argmin(dists)
        return index_nn, self.tree[index_nn].coordinates

    def nearest_neighbour(self, points, q):
        """
        Returns the index in the tree that is nearest to qrand.
        Also returns the coordinates of the nearest index
        :param qrand:
        :return:
        """
        dists = np.linalg.norm(points - q, axis=1)
        index_nn = np.argmin(dists)
        return index_nn, points[index_nn, :]

    def closest_node_to_goals(self):
        """
        Returns the index in the tree that is nearest to qrand
        :param qrand:
        :return:
        """
        distances = []
        indices_nn = []
        for i in range(self.goals.shape[0]):
            index_nn, coords_nn = self.nearest_neighbour_in_tree(self.goals[i])
            d = self.distance(coords_nn, self.goals[i])
            distances.append(d)
            indices_nn.append(index_nn)
        i = np.argmin(np.array(distances))
        j = indices_nn[i]
        return self.tree[j]

    def closest_goal_to_node(self, node):
        """
        Returns the index in the tree that is nearest to qrand
        :param qrand:
        :return:
        """
        distances = []
        for i in range(self.goals.shape[0]):
            d = self.distance(node.coordinates, self.goals[i])
            distances.append(d)
        i = np.argmin(np.array(distances))
        return self.goals[i]

    def reached_goal(self, q):
        """
        Checks if the coordinates in q have reached any of the goals. A goal is reached if q is within a radius epsilon
        around any of the goals.
        CAUTION: only considering 2D coordinates
        :param q:
        :return:
        """
        for i in range(self.goals.shape[0]):
            # d = np.linalg.norm(q-self.goals[i])
            d = self.distance(q[0:2], self.goals[i, 0:2])
            if d < self.epsilon:
                return True
        return False

    def distance(self, qa, qb):
        d = np.linalg.norm(qa-qb)
        return d

    def get_path(self):
        """
        This method:
        a) looks for the node/nodes that are marked as "reached the goal"
        b) for each node marked as reached, backtraces the path until the root is found.
        c) The path is returned.
        If no node is found as goal_reached, then the closest node to the goal is found and treated as the solution node
        :return:
        """
        solution_node = self.find_node_reached()
        # if no solution is found, then find the nearest neighbour in the tree to the goal
        if solution_node is None:
            solution_node = self.closest_node_to_goals()
            solution_path = self.back_trace_path_from_node(solution_node)
        else:
            # standard solution
            solution_path = self.back_trace_path_from_node(solution_node)
            goal = self.closest_goal_to_node(solution_node)
            # in this case (reached goal), the goal coordinates are inserted at the beginning
            solution_path.insert(0, np.array(goal))

        # change order of the path!!
        solution_path.reverse()
        solution_path = np.array(solution_path)
        return solution_path

    def find_node_reached(self):
        """
        Finds the node that reached the target. We must iterate through all the nodes in the tree. However, doing
        it in reverse order is probably faster
        :return:
        """
        for i in reversed(range(len(self.tree))):
            if self.tree[i].goal_reached:
                return self.tree[i]
        return None

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
        print('PERFORMED ITERATIONS: ', self.iterations_performed)
        print('GOAL REACHED: ', self.goal_reached)
        print(30 * '*')

    def plot_tree(self, show_obstacles=False, show_traversable=False, show=False):
        """
        Plots the 3D tree, the obstacles and traversable points
        """
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        if show_obstacles:
            # plot obstacles
            ax.scatter(self.pc_obstacles[:, 0], self.pc_obstacles[:, 1], self.pc_obstacles[:, 2],
                   marker='.', color='red')
        if show_traversable:
            # plot traversable points
            ax.scatter(self.pc_traversable[:, 0], self.pc_traversable[:, 1], self.pc_traversable[:, 2],
                   marker='.', color='green')

        # plot the edges of the tree
        for i in range(len(self.tree)):
            coords = self.tree[i].coordinates
            parent_id = self.tree[i].parent_id
            coords_parent = self.tree[parent_id].coordinates
            c = np.vstack((coords, coords_parent))
            ax.scatter(c[:, 0], c[:, 1], c[:, 2], marker='.', color='magenta')
        # plot vertices (node coordinates) as scatter
        coords = []
        # leave out i=0, since it corresponds to the start node
        for i in range(1, len(self.tree)):
            coords.append(self.tree[i].coordinates)
        coords = np.array(coords)
        # plot all nodes in the tree
        plt.scatter(coords[:, 0], coords[:, 1], coords[:, 2], marker='s', color='blue')

        # plot start
        plt.scatter(self.start[0], self.start[1], self.start[2], marker='s', color='green')
        # plot goals
        for i in range(self.goals.shape[0]):
            goal = self.goals[i]
            plt.scatter(goal[0], goal[1], goal[2], color='red')

        if show:
            plt.show()

    def plot_tree2d(self, show_obstacles=False, show=False):
        """
        Plots a 2D tree, the obstacles and traversable points
        """
        if show_obstacles:
            # plot obstacles
            plt.scatter(self.pc_obstacles[:, 0], self.pc_obstacles[:, 1],
                        marker='s', color='red')
        # if show_traversable:
        #     # plot traversable points
        #     plt.scatter(self.pc_traversable[:, 0], self.pc_traversable[:, 1],
        #                 marker='.', color='green')

        # plot the edges of the tree
        for i in range(len(self.tree)):
            coords = self.tree[i].coordinates
            parent_id = self.tree[i].parent_id
            coords_parent = self.tree[parent_id].coordinates
            c = np.vstack((coords, coords_parent))
            plt.plot(c[:, 0], c[:, 1], c[:, 2], marker='.', color='magenta')
        # plot vertices (node coordinates) as scatter
        coords = []
        # leave out i=0, since it corresponds to the start node
        for i in range(1, len(self.tree)):
            coords.append(self.tree[i].coordinates)
        coords = np.array(coords)
        # plot all nodes in the tree
        plt.scatter(coords[:, 0], coords[:, 1], marker='o', color='blue')

        # plot start
        plt.scatter(self.start[0], self.start[1], marker='s', color='green')
        # plot goals
        for i in range(self.goals.shape[0]):
            goal = self.goals[i]
            plt.scatter(goal[0], goal[1], color='green')

        if show:
            plt.show()

    def plot_solution(self, show=True):
        solution_path = self.get_path()
        solution_path = np.array(solution_path)
        K = solution_path.shape[0]
        for i in range(K-1):
            coords = solution_path[i, :]
            coordsn = solution_path[i+1, :]
            c = np.vstack((coords, coordsn))
            plt.plot(c[:, 0], c[:, 1], color='cyan')
        if show:
            plt.show()

    def plot_traversable(self):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        # plot traversable points
        ax.scatter(self.pc_traversable[:, 0], self.pc_traversable[:, 1], self.pc_traversable[:, 2],
                   marker='.', color='green')
        plt.show()

    def plot_obstacle(self):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        # plot traversable points
        ax.scatter(self.pc_obstacles[:, 0], self.pc_obstacles[:, 1], self.pc_obstacles[:, 2],
                   marker='.', color='red')
        plt.show()

    def plot_all(self):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        # plot traversable points
        ax.scatter(self.pc_traversable[:, 0], self.pc_traversable[:, 1], self.pc_traversable[:, 2],
                   marker='.', color='green')
        ax.scatter(self.pc_obstacles[:, 0], self.pc_obstacles[:, 1], self.pc_obstacles[:, 2],
                   marker='.', color='red')
        plt.show()


class Node():
    """
    A node in the tree, storing its id, parent_id and coordinates.
    """
    def __init__(self, id, parent_id, coordinates, reached=False):
        self.id = id
        self.parent_id = parent_id
        self.coordinates = coordinates
        self.goal_reached = reached
