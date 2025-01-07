import time
from typing import Tuple
from entity import Entity
from vector import Vector
import tkinter as tk


class Goal(Entity):
    """
    Represents a goal that can be drawn on a Tkinter Canvas.
    """

    # Class attributes for default styling
    EmojiFont = ("Arial", 30)
    NumberFont = ("Arial", 15)

    # State representations
    StateColors = {
        "Free": "red",
        "Assigned": "orange",
        "Visited": "green",
    }

    StateEmojis = {
        "Free": "❌",
        "Assigned": "⌛",
        "Visited": "✅",
    }

    def __init__(self, pos: Vector = None, goal_no: int = 1, state: str = "Free"):
        """
        Initializes the Goal object.

        :param pos: Position as a Vector.
        :param goal_no: Goal number (must be positive integer).
        :param state: Initial state of the goal ("Free", "Assigned", or "Visited").
        """
        self.pos = pos if pos is not None else Vector(0.0, 0.0)
        self.goal_no = goal_no
        self.state = state
        # Initialize canvas item IDs
        self.number_id = None
        self.emoji_id = None
        self.last_visited_time = time.time()  # Initialize to None

    @property
    def goal_no(self) -> int:
        return self._goal_no

    @goal_no.setter
    def goal_no(self, value: int):
        if not isinstance(value, int) or value <= 0:
            raise ValueError("Goal number must be a positive integer.")
        self._goal_no = value

    @property
    def state(self) -> str:
        return self._state

    @state.setter
    def state(self, value: str):
        if value not in self.StateColors:
            valid_states = ", ".join(self.StateColors.keys())
            raise ValueError(
                f"Invalid state: {value}. Valid states are: {valid_states}."
            )
        self._state = value
        self.goal_color = self.StateColors[value]
        self.goal_emoji = self.StateEmojis.get(value, "❌")

    def draw(self, canvas: tk.Canvas) -> None:
        """
        Draws or updates the goal on the provided Tkinter Canvas.

        :param canvas: The Tkinter Canvas where the goal will be drawn.
        """
        x, y = self.pos.x, self.pos.y

        # Draw the goal number above the emoji
        self.number_id = canvas.create_text(
            x,
            y - 30,
            text=f"Goal {self.goal_no}",
            fill=self.goal_color,
            font=self.NumberFont,
        )

        # Draw the emoji at the center
        self.emoji_id = canvas.create_text(
            x,
            y,
            text=self.goal_emoji,
            fill=self.goal_color,
            font=self.EmojiFont,
        )

        # Update existing canvas items
        canvas.itemconfigure(
            self.number_id, text=f"Goal {self.goal_no}", fill=self.goal_color
        )
        canvas.itemconfigure(self.emoji_id, text=self.goal_emoji, fill=self.goal_color)

    def on_click(self, event):
        """
        Handles click events on the goal.
        """
        print(f"Goal {self.goal_no} clicked.")

    def bind_events(self, canvas: tk.Canvas) -> None:
        """
        Binds events to the goal's canvas items.

        :param canvas: The Tkinter Canvas where the goal is drawn.
        """
        canvas.tag_bind(self.emoji_id, "<Button-1>", self.on_click)
        canvas.tag_bind(self.number_id, "<Button-1>", self.on_click)
    
    def update_state(self, current_time, t_threshold1):
        """
        Updates the goal's state based on the time since it was last visited.
        """
        if self.state == "Visited" and self.last_visited_time is not None:
            elapsed_time = current_time - self.last_visited_time
            if elapsed_time >= t_threshold1:
                #self.state = "Free"
                #self.last_visited_time = None  # Reset last visited time
                pass
    
    @state.setter
    def state(self, value: str):
        if value not in self.StateColors:
            valid_states = ", ".join(self.StateColors.keys())
            raise ValueError(f"Invalid state: {value}. Valid states are: {valid_states}.")
        self._state = value
        self.goal_color = self.StateColors[value]
        self.goal_emoji = self.StateEmojis.get(value, "❌")
        if hasattr(self, "canvas"):  # Redraw if canvas exists
            self.draw(self.canvas)
    
    @staticmethod
    def get_font(size: int) -> Tuple[str, int]:
        return "Arial", size
