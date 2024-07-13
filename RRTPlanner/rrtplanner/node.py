"""
The node class.
"""


class Node():
    """
    A node in the tree, storing its id, parent_id and coordinates.
    """
    def __init__(self, id, parent_id, coordinates, goal_reached=False):
        self.id = id
        self.parent_id = parent_id
        self.coordinates = coordinates
        self.goal_reached = goal_reached
