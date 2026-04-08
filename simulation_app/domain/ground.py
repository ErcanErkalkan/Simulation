import tkinter as tk

from simulation_app.domain.entity import Entity
from simulation_app.domain.vector import Vector


class Ground(Entity):
    """
    Represents a home icon that can be drawn on a Tkinter Canvas.
    """

    EmojiFont = ("Arial", 50)
    DefaultColor = "black"

    def __init__(self, pos: Vector = None, color: str = DefaultColor):
        """
        Initializes a new instance of the Home class.

        :param pos: Position of the home icon (Vector).
        :param color: Color of the home icon.
        """
        self.pos = pos if pos is not None else Vector(0.0, 0.0)
        self.color = color

    def draw(self, canvas: tk.Canvas) -> None:
        """
        Draws the home icon on the specified Tkinter Canvas.

        :param canvas: The Tkinter Canvas where the home will be drawn.
        """
        x, y = self.pos.x, self.pos.y

        # Draw the home emoji
        home_emoji = "🏠"
        canvas.create_text(
            x,
            y,
            text="\U0001F3E0",
            fill=self.color,
            font=self.EmojiFont,
            anchor="center"
        )
