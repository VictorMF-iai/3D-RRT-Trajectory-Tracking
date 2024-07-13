"""
    The main class RRTPlanner and the node class
    This constitutes a demo of a 2D planning of a holonomic robot with circle-like obstacles.
    A set of circle-like obstacles are considered.

    Please, read:
    - RRT-connect: An efficient approach to single-query path planning. J. J. Kuffner and S. M. LaValle. In Proceedings IEEE International Conference on Robotics and Automation, pages 995-1001, 2000. [pdf].
    - Rapidly-exploring random trees: A new tool for path planning. S. M. LaValle. TR 98-11, Computer Science Dept., Iowa State University, October 1998, [pdf].

    The minor variation with respect to the basic algorithm is that, after max_nodes iterations, the
    algorithm always returns a solution:
        a) If the goal is reached, the path that connects the start and goal is returned.
        b) If the goal is not reached, the node that is found closest to the goal is found.
           Next, the path that connects the start and the closest node is returned.
    This behaviour is not optimal, however, it yields, sometimes, necessary to have always
    a local path to follow in the context of mobile robotics. It yields probable, that, in the next
    observations, a valid path is found that allows the robot to reach the goal.
"""
import numpy as np
import matplotlib.pyplot as plt
from rrtplanner.tree import Tree

REACHED = 0
ADVANCED = 1
TRAPPED = 2
GOAL_REACHED = 3


class RRTPlanner():
    """
    A Basic RRT Planner in a 2D space for holonomic robots.
    """
    def __init__(self, start, goal, dimensions, obstacles, epsilon, max_nodes):
        self.goal = np.array(goal)
        self.start = np.array(start)
        self.max_nodes = max_nodes
        self.epsilon = epsilon
        # space, +- dimensions in XY
        self.dim = np.array(dimensions)
        # a set of circle-like obstacles, (x, y, R)
        self.obstacles = np.array(obstacles)
        # stores info on the result of the algorithm
        self.goal_reached = False
        self.iterations_performed = 0

        if self.collision(self.start) or self.collision(self.goal):
            raise Exception('Start or goal are inside the obstacle space')

    def build_rrt_basic(self):
        """
        Programmed using the same names as in:
        RRT-connect: An efficient approach to single-query path planning.
        J. J. Kuffner and S. M. LaValle.
        In Proceedings IEEE International Conference on Robotics and Automation,
        pages 995-1001, 2000.
        """
        treeA = Tree(self.start)
        for k in range(self.max_nodes):
            print('Iteration: ', k)
            qrand = self.random_config()
            result, qnew = self.extend(treeA, qrand)
            if self.reached_goal(qnew):
                self.goal_reached = True
                self.iterations_performed = k
                break
        return treeA

    def build_rrt_connect(self):
        """
        Programmed using the same names as in:
        RRT-connect: An efficient approach to single-query path planning.
        J. J. Kuffner and S. M. LaValle.
        In Proceedings IEEE International Conference on Robotics and Automation,
        pages 995-1001, 2000.
        """
        # use two trees in this case
        t1 = Tree(self.start)
        t2 = Tree(self.goal)
        tA = t1
        tB = t2
        for k in range(self.max_nodes):
            print('Iteration: ', k)
            qrand = self.random_config()
            result, qnew = self.extend(tA, qrand)
            if not (result == TRAPPED):
                result, qfinal = self.connect(tB, qnew)
                # in RRT connect, a REACHED, means that both trees could be connected.
                if (result == REACHED) or (result == GOAL_REACHED):
                    # In this case, we mark the two nodes as reached (in both trees)
                    nodeneartA = tA.nearest_neighbour(qfinal)
                    tA.mark_node_as_reached(nodeneartA.id)
                    nodeneartB = tB.nearest_neighbour(qfinal)
                    tB.mark_node_as_reached(nodeneartB.id)
                    self.goal_reached = True
                    self.iterations_performed = k
                    return t1, t2
            # swap trees
            tA, tB = self.swap(tA, tB)
        return t1, t2

    def build_rrt_connect_to_goal(self, try_obvious=True):
        """
        This is a simplified version of the RRT-connect algorithm using a single
        tree (see references). The connect operation always tries to connect the tree to the goal.
        The try_obvious option, directs the search to the goal in the first iteration. This allows
        to have nice behaviour, since, whenever the space is free, the obvious line is found between
        the start and goal positions.

        RRT-connect: An efficient approach to single-query path planning.
        J. J. Kuffner and S. M. LaValle.
        In Proceedings IEEE International Conference on Robotics and Automation,
        pages 995-1001, 2000.
        """
        # use a single tree in this case
        # no swap operation needed
        tree = Tree(self.start)
        for k in range(self.max_nodes):
            print('Iteration: ', k)
            if k == 0 and try_obvious:
                qrand = self.goal
            else:
                qrand = self.random_config()
            result, qnew = self.extend(tree, qrand)
            if not (result == TRAPPED):
                result, qfinal = self.connect(tree, self.goal)
                # in RRT connect, a REACHED, means that both trees could be connected.
                if (result == REACHED) or (result == GOAL_REACHED):
                    # In this case, we mark the two nodes as reached (in both trees)
                    self.goal_reached = True
                    self.iterations_performed = k
                    return tree
        return tree

    def random_config(self):
        """
        This generates a new random configuration
        :return:
        """
        x = -self.dim[0] + 2*self.dim[0]*np.random.rand()
        y = -self.dim[1] + 2*self.dim[1] * np.random.rand()
        return np.array([x, y])

    def extend(self, tree, q):
        """
        The extend operation of the tree.
        A result is added to this operation to indicate that qnew reached the goal
        The REACHED result is not considered in this case
        :param qrand:
        :return:
        """
        node_near = self.nearest_neighbour(tree, q)
        qnew = self.new_config(node_near.coordinates, q)
        if not self.collision(qnew):
            goal_reached = self.reached_goal(qnew)
            reached_new_config = self.distance(qnew, q) < self.epsilon
            # if goal_reached:
            #     tree.add_vertex(parent_id=node_near.id, coordinates=qnew, goal_reached=True)
            #     return GOAL_REACHED, qnew
            if reached_new_config:
                tree.add_vertex(parent_id=node_near.id, coordinates=qnew, goal_reached=goal_reached)
                return REACHED, qnew
            else:
                tree.add_vertex(parent_id=node_near.id, coordinates=qnew, goal_reached=goal_reached)
                return ADVANCED, qnew
        return TRAPPED, qnew

    def connect(self, tree, q):
        while True:
            result, qfinal = self.extend(tree, q)
            if not (result == ADVANCED):
                break
        return result, qfinal

    def new_config(self, qnear, qrand):
        """
        Computes a new configuration between qnear and qrand that is placed at an epsilon distance from qnear
        :param qnear:
        :param qrand:
        :return:
        """
        ds = qrand - qnear
        phi = np.arctan2(ds[1], ds[0])
        du = np.array([np.cos(phi), np.sin(phi)])
        qnew = qnear + self.epsilon*du
        return qnew

    def collision(self, q):
        """
        Returns True if q is in the obstacle space (it is colliding), false elsewise
        :param q:
        :return:
        """
        for i in range(len(self.obstacles)):
            obstacle = self.obstacles[i]
            d = np.linalg.norm(q-obstacle[0:2])
            if d < obstacle[2]:
                return True
        return False

    def nearest_neighbour(self, tree, qrand):
        """
        Returns the index in the tree that is nearest to qrand
        :param qrand:
        :return:
        """
        node = tree.nearest_neighbour(qrand)
        return node

    def reached_goal(self, q):
        d = self.distance(q, self.goal)
        if d < self.epsilon:
            return True
        return False

    def distance(self, qa, qb):
        d = np.linalg.norm(qa - qb)
        return d

    def get_solution_path(self, tree):
        """
        This method:
        a) looks for the node/nodes that are marked as "reached the goal"
        b) for each node marked as reached, backtraces the path until the root is found.
        c) The path is returned.
        If no node is found as goal_reached, then the closest node to the goal is found and treated as the solution node
        :return:
        """
        solution_node = tree.find_node_reached()
        # if no solution is found, then find the nearest neighbour in the tree to the goal
        if solution_node is None:
            solution_node = self.nearest_neighbour(tree, self.goal)
            solution_path = tree.back_trace_path_from_node(solution_node)
        else:
            # standard solution
            solution_path = tree.back_trace_path_from_node(solution_node)
            # in this case (reached goal), the goal coordinates are inserted at the beginning
            solution_path.insert(0, np.array(self.goal))
        # change order of the path!!
        solution_path.reverse()
        solution_path = np.array(solution_path)
        return solution_path

    def swap(self, tA, tB):
        # swap trees
        temp = tA
        tA = tB
        tB = temp
        return tA, tB

    def get_solution_path_from_two_trees(self, treeA, treeB):
        """
        This method looks for the global path that connects the two trees.
        Several cases appear, in the most common, both trees have one of the
        nodes marked as reached. This means that they reached the other tree.

        treeA is assumed to be rooted by the start position.
        treeB is assumed to be rooted by the goal position position.

        :return:
        """
        solution_nodeA = treeA.find_node_reached()
        solution_nodeB = treeB.find_node_reached()
        # standard case
        if (solution_nodeA is not None) and (solution_nodeB is not None):
            # standard solution
            solution_pathA = treeA.back_trace_path_from_node(solution_nodeA)
            solution_pathA.reverse()
            solution_pathB = treeB.back_trace_path_from_node(solution_nodeB)
            solution_pathA = np.array(solution_pathA)
            solution_pathB = np.array(solution_pathB)
            solution_path = np.vstack((solution_pathA, solution_pathB))
        # no solution was found
        if (solution_nodeA is None) or (solution_nodeB is None):
            print('NO SOLUTION COULD BE FOUND')
            solution_path = []
            solution_path = np.array(solution_path)
        return solution_path

    def print_info(self):
        print(30*'*')
        print('PERFORMED ITERATIONS: ', self.iterations_performed)
        print('GOAL REACHED: ', self.goal_reached)
        print(30 * '*')

    def plot(self, show=False):
        # plot obstacles
        theta = np.linspace(0, 2 * np.pi, 150)
        xobs = np.array([])
        yobs = np.array([])
        for i in range(len(self.obstacles)):
            obstacle = self.obstacles[i]
            x = obstacle[2] * np.cos(theta) + obstacle[0]
            y = obstacle[2] * np.sin(theta) + obstacle[1]
            xobs = np.append(xobs, x)
            yobs = np.append(yobs, y)
        obspoints = np.column_stack((xobs, yobs))
        plt.scatter(obspoints[:, 0], obspoints[:, 1], color='black')
        plt.scatter(self.start[0], self.start[1], color='green')
        plt.scatter(self.goal[0], self.goal[1], color='red')

