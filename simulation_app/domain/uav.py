import time
from typing import Optional, Set

import tkinter as tk

from simulation_app.domain.entity import Entity
from simulation_app.domain.goal import Goal
from simulation_app.domain.ground import Ground
from simulation_app.domain.vector import Vector


class UAV(Entity):
    """
    Represents a UAV that can move, interact with goals, and be drawn on a canvas.
    """

    UAVFont = ("Arial", 40)
    NumberFont = ("Arial", 15)

    StateColors = {
        "Free": "blue",
        "Leader": "green",
        "Slave": "orange",
        "Ground_Leader": "darkgreen",
        "Relay": "purple",
    }

    def __init__(
        self,
        pos: Optional[Vector] = None,
        direction: Optional[Vector] = None,
        uav_no: int = 1,
        ground: Optional[Ground] = None,
    ):
        self.pos: Vector = pos if pos is not None else Vector(0, 0)
        self.direction: Vector = direction if direction is not None else Vector(1, 0)
        self.uav_no: int = uav_no
        self.state: str = "Free"
        self.target: Optional[Goal] = None
        self.relay_position: Optional[Vector] = None
        self.my_leader: Optional["UAV"] = None
        self.my_slave_list: Set["UAV"] = set()
        self.my_relay_list: Set["UAV"] = set()
        self.ground: Optional[Ground] = ground
        self.uav_id: Optional[int] = None
        self.label_id: Optional[int] = None
        self.running = False
        self.speed = 5.0

    def move_to_target(self, target: Optional[Goal] = None) -> None:
        if target is not None:
            self.target = target
        if self.target is None:
            return

        next_pos = self.get_next_position(self.target.pos)
        reached_target = next_pos == self.target.pos
        self.pos = next_pos

        if not reached_target:
            return

        self.target.state = "Visited"
        self.target.last_visited_time = time.time()
        self.target = None
        if self.state == "Ground_Leader":
            self.delete_my_relay_list()
        self.delete_my_slave_list()
        self.state = "Free"

    def move(self, target_pos: Vector) -> None:
        self.pos = self.get_next_position(target_pos)

    def move_as_relay(self) -> None:
        if self.relay_position is not None:
            self.move_to_position(self.relay_position)

    def move_to_position(self, target_pos: Vector) -> None:
        self.pos = self.get_next_position(target_pos)

    def move_to_ground(self) -> None:
        if self.ground is not None:
            self.move(self.ground.pos)

    def move_to_leader(self) -> None:
        if self.my_leader is not None:
            self.pos = self.get_next_position(self.my_leader.pos)

    def get_leader(self):
        if self.state in {"Slave", "Relay"}:
            return self.my_leader
        return None

    def delete_my_slave_list(self) -> None:
        if self.my_leader is not None:
            self.my_leader.my_slave_list.discard(self)
            self.my_leader.my_relay_list.discard(self)
            self.my_leader = None
        for slave in list(self.my_slave_list):
            slave.state = "Free"
            slave.my_leader = None
        self.my_slave_list.clear()

    def delete_my_relay_list(self) -> None:
        for relay in list(self.my_relay_list):
            relay.state = "Free"
            relay.my_leader = None
        self.my_relay_list.clear()

    def stop(self) -> None:
        self.running = False

    def draw(self, canvas: tk.Canvas) -> None:
        x, y = self.pos.x, self.pos.y
        fill_color = self.StateColors.get(self.state, "gray")

        self.uav_id = canvas.create_text(
            x,
            y,
            text="\U0001F681",
            fill=fill_color,
            font=self.UAVFont,
            anchor="center",
        )

        self.label_id = canvas.create_text(
            x,
            y - 35,
            text=f"UAV {self.uav_no} {self.state}",
            fill=fill_color,
            font=self.NumberFont,
            anchor="center",
        )

        canvas.coords(self.uav_id, x, y)
        canvas.itemconfigure(self.uav_id, fill=fill_color)
        canvas.coords(self.label_id, x, y - 35)
        canvas.itemconfigure(
            self.label_id,
            text=f"UAV {self.uav_no} {self.state}",
            fill=fill_color,
        )

    def __str__(self) -> str:
        return (
            f"UAV {self.uav_no}:\n"
            f"  State: {self.state}\n"
            f"  Position: {self.pos}\n"
            f"  Direction: {self.direction}\n"
            f"  Target: {self.target.pos if self.target else 'None'}\n"
            f"  Leader: UAV {self.my_leader.uav_no if self.my_leader else 'None'}\n"
            f"  Slaves: {[slave.uav_no for slave in self.my_slave_list]}\n"
            f"  Relays: {[relay.uav_no for relay in self.my_relay_list]}\n"
            f"  Ground Station: {self.ground.pos if self.ground else 'None'}"
        )

    def get_next_position(self, target_pos: Vector) -> Vector:
        direction = Vector(target_pos.x - self.pos.x, target_pos.y - self.pos.y)
        distance = direction.length()
        if distance == 0:
            return Vector(self.pos.x, self.pos.y)
        if distance <= self.speed:
            return Vector(target_pos.x, target_pos.y)

        unit_direction = direction / distance
        return Vector(
            self.pos.x + unit_direction.x * self.speed,
            self.pos.y + unit_direction.y * self.speed,
        )
