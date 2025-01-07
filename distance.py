from entity import Entity
from vector import Vector
import math


class Distance:
    """Operations for calculating distances between entities or positions."""

    @staticmethod
    def distance_between(obj1: Entity, obj2: Entity) -> float:
        """
        Calculates the Euclidean distance between two entities.
        
        :param obj1: The first entity with a 'pos' attribute of type Vector.
        :param obj2: The second entity with a 'pos' attribute of type Vector.
        :return: The Euclidean distance between the two entities.
        """
        return Distance.distance_between_positions(obj1.pos, obj2.pos)

    @staticmethod
    def distance_between_positions(pos1: Vector, pos2: Vector) -> float:
        """
        Calculates the Euclidean distance between two positions.
        
        :param pos1: The first position as a Vector.
        :param pos2: The second position as a Vector.
        :return: The Euclidean distance between the two positions.
        """
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        return math.hypot(dx, dy)
