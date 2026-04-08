from __future__ import annotations

import tkinter as tk
from typing import Optional, Tuple

from simulation_app.domain.entity import Entity
from simulation_app.domain.vector import Vector


class Goal(Entity):
    """
    Represents a goal that can be drawn on a Tkinter Canvas.
    """

    EmojiFont = ("Arial", 30)
    NumberFont = ("Arial", 15)

    StateColors = {
        "Free": "red",
        "Assigned": "orange",
        "Visited": "green",
    }

    StateEmojis = {
        "Free": "\u274c",
        "Assigned": "\u231b",
        "Visited": "\u2705",
    }

    def __init__(
        self,
        pos: Optional[Vector] = None,
        goal_no: int = 1,
        state: str = "Free",
    ):
        self.pos = pos if pos is not None else Vector(0.0, 0.0)
        self.goal_no = goal_no
        self.number_id = None
        self.emoji_id = None
        self.last_visited_time = None
        self.state = state

    @property
    def goal_no(self) -> int:
        return self._goal_no

    @goal_no.setter
    def goal_no(self, value: int) -> None:
        if not isinstance(value, int) or value <= 0:
            raise ValueError("Goal number must be a positive integer.")
        self._goal_no = value

    @property
    def state(self) -> str:
        return self._state

    @state.setter
    def state(self, value: str) -> None:
        if value not in self.StateColors:
            valid_states = ", ".join(self.StateColors.keys())
            raise ValueError(
                f"Invalid state: {value}. Valid states are: {valid_states}."
            )
        self._state = value
        self.goal_color = self.StateColors[value]
        self.goal_emoji = self.StateEmojis[value]

    def draw(self, canvas: tk.Canvas) -> None:
        x, y = self.pos.x, self.pos.y
        self.number_id = canvas.create_text(
            x,
            y - 30,
            text=f"Goal {self.goal_no}",
            fill=self.goal_color,
            font=self.NumberFont,
        )
        self.emoji_id = canvas.create_text(
            x,
            y,
            text=self.goal_emoji,
            fill=self.goal_color,
            font=self.EmojiFont,
        )

    def on_click(self, event) -> None:
        print(f"Goal {self.goal_no} clicked.")

    def bind_events(self, canvas: tk.Canvas) -> None:
        if self.emoji_id is not None:
            canvas.tag_bind(self.emoji_id, "<Button-1>", self.on_click)
        if self.number_id is not None:
            canvas.tag_bind(self.number_id, "<Button-1>", self.on_click)

    def update_state(self, current_time: float, t_threshold1: float) -> None:
        if self.state == "Visited" and self.last_visited_time is not None:
            elapsed_time = current_time - self.last_visited_time
            if elapsed_time >= t_threshold1:
                self.state = "Free"

    @staticmethod
    def get_font(size: int) -> Tuple[str, int]:
        return "Arial", size
