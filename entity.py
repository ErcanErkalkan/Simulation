from vector import Vector
from idrawable import IDrawable
import tkinter as tk

class Entity(IDrawable):
    def __init__(self, x: float, y: float, color: str = "black", size: float = 10):
        """
        Initializes the entity with position, color, and size.

        :param x: X-coordinate of the entity.
        :param y: Y-coordinate of the entity.
        :param color: The color of the entity when drawn (default is black).
        :param size: The size of the entity when drawn (default is 10).
        """
        self.pos = Vector(x, y)
        self.color = color
        self.size = size

    def distance_to(self, other: 'Entity') -> float:
        """
        Calculates the Euclidean distance to another entity.

        :param other: The other entity to calculate distance to.
        :return: The Euclidean distance between the two entities.
        :raises TypeError: If the 'other' is not an Entity.
        """
        if not isinstance(other, Entity):
            raise TypeError("Argument 'other' must be an instance of Entity.")
        return self.pos.distance_to(other.pos)

    def translate(self, dx: float, dy: float):
        """
        Translates (moves) the entity by a given offset.

        :param dx: Change in X-coordinate.
        :param dy: Change in Y-coordinate.
        """
        self.pos = Vector(self.pos.x + dx, self.pos.y + dy)

    def draw(self, canvas: tk.Canvas):
        """
        Draws the entity as a circle on the canvas.

        :param canvas: The Tkinter canvas to draw on.
        """
        pass
    
    def __str__(self):
        return f"Entity(Position: {self.pos}, Color: {self.color}, Size: {self.size})"

    def __repr__(self):
        return self.__str__()
