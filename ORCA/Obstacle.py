from Vector2 import *


class Obstacle:
    """
    Defines static obstacles in the simulation.
    """
    def __init__(self):
        """
        Constructs a static obstacle instance.
        """
        self.direction_ = Vector2()
        self.point_ = Vector2()
        self.next_ = None  # Obstacle reference
        self.previous_ = None  # Obstacle reference
        self.id_ = 0  # Obstacle ID
        self.isConvex_ = False  # Whether the obstacle is convex




