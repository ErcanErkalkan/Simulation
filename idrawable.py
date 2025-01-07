from abc import ABC, abstractmethod
import tkinter as tk


class IDrawable(ABC):
    """
    Represents an object that can be drawn on a Tkinter Canvas.
    """

    @abstractmethod
    def draw(self, canvas):
        """
        Draws the object on the specified Tkinter Canvas.

        :param canvas: The Tkinter Canvas where the object will be drawn.
        """
        pass
