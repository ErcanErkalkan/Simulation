import time
from typing import Set, Optional, List
from entity import Entity
from vector import Vector
from goal import Goal
from ground import Ground
import tkinter as tk


class UAV(Entity):
    """
    Represents a UAV (Unmanned Aerial Vehicle) that can move, interact with goals,
    and be drawn on a Tkinter canvas.
    """

    UAVFont = ("Arial", 40)  # Font size for rendering the UAV emoji
    NumberFont = ("Arial", 15)  # Font size for displaying the UAV number

    StateColors = {
        "Free": "blue",  # Free state color
        "Leader": "green",  # Leader state color
        "Slave": "orange",  # Slave state color
    }

    def __init__(
        self,
        pos: Optional[Vector] = None,
        direction: Optional[Vector] = None,
        uav_no: int = 1,
        ground: Optional[Ground] = None,
    ):
        """
        Initializes a UAV.

        :param pos: Position of the UAV as a Vector.
        :param direction: Direction of the UAV as a Vector.
        :param uav_no: Unique identifier for the UAV.
        :param ground: Reference to the Ground object.
        """
        self.pos: Vector = pos if pos else Vector(0, 0)
        self.direction: Vector = direction if direction else Vector(1, 0)
        self.uav_no: int = uav_no
        self.state: str = "Free"  # Possible states: "Free", "Leader", "Slave"
        self.target: Optional[Goal] = None
        self.my_leader: Optional["UAV"] = None
        self.my_slave_list: Set["UAV"] = set()  # Using a set for unique slaves
        self.ground: Optional[Ground] = ground

        # Canvas item IDs for updating or binding events
        self.uav_id: Optional[int] = None
        self.label_id: Optional[int] = None
        self.running = False  # İş parçacığı kontrolü için bayrak
        self.speed = 1  # UAV's movement speed

    def move_to_target(self, target: Optional[Goal] = None):
        """
        Moves the UAV toward a specified target or in its current direction.

        :param target: Target position as a Vector. If None, moves in its current direction.
        """
        if target:
            self.target = target
        if self.target:
            direction_to_target = self.target.pos - self.pos
            distance = direction_to_target.length()
            if distance < 1:  # If the UAV is close enough to the goal
                self.pos = self.target.pos  # Snap to the goal
                self.state = "Free"  # Update UAV state to Free
                self.target.state = "Visited"  # Update Goal state to Visited
                self.target.last_visited_time = time.time()
                self.target = None  # Clear the target
                self.delete_my_slave_list()
            else:
                self.pos += direction_to_target.normalize()

    def move(self, target_pos: Vector):
        """
        Moves the UAV toward the specified position.

        :param target_pos: The target position to move towards.
        """
        direction_to_target = target_pos - self.pos
        distance = direction_to_target.length()
        if distance < 1:
            self.pos = target_pos
        else:
            self.pos += direction_to_target.normalize()
    
    # In UAV class
    def move_to_position(self, target_pos: Vector):
        """
        Move the UAV towards a specified position.
        """
        next_pos = self.get_next_position(target_pos)
        self.pos = next_pos


    def move_to_ground(self):
        """
        Moves the UAV toward its associated ground station.
        """
        if self.ground:
            self.move(self.ground.pos)

    def move_to_leader(self):
        """
        Makes the UAV follow its leader, if one exists.
        """
        if self.my_leader:
            direction_to_my_leader = self.my_leader.pos - self.pos
            distance = direction_to_my_leader.length()
            if distance < 1:  # If the UAV is close enough to the goal
                self.pos = self.my_leader.pos  # Snap to the goal
            else:
                self.pos += direction_to_my_leader.normalize()

    def get_leader(self):
        if self.state == "Slave":
            return self.my_leader
        else:
            return None

    def _delete_my_slave_list(self):
        """
        Clears all hierarchical relationships and sets the UAV state to "Free".
        """
        for slave in self.my_slave_list:
            slave.state = "Free"
            slave.my_leader = None
        self.my_slave_list.clear()
    
        # In the UAV class
    def delete_my_slave_list(self):
        # Remove self from leader's slave list if applicable
        if self.my_leader:
            self.my_leader.my_slave_list.discard(self)
            self.my_leader = None
        # Clear own slave list
        self.my_slave_list.clear()


    def stop(self):
        self.running = False

    def draw(self, canvas: tk.Canvas):
        """
        Draws the UAV on the Tkinter canvas using an emoji.

        :param canvas: Tkinter Canvas to draw on.
        """
        x, y = self.pos.x, self.pos.y

        # Determine the color and emoji based on the UAV state
        fill_color = self.StateColors.get(self.state, "gray")  # Default color: gray
        uav_emoji = "🚁"  # Drone emoji

        # Draw the drone emoji
        self.uav_id = canvas.create_text(
            x,
            y,
            text=uav_emoji,
            fill=fill_color,
            font=self.UAVFont,
            anchor="center",
        )

        # Draw the UAV's label above the emoji
        self.label_id = canvas.create_text(
            x,
            y - 35,  # Slightly above the emoji
            text=f"UAV {self.uav_no} {self.state}",
            fill=fill_color,
            font=self.NumberFont,
            anchor="center",
        )

        # Update the UAV emoji position and color
        canvas.coords(self.uav_id, x, y)
        canvas.itemconfigure(self.uav_id, fill=fill_color)

        # Update the UAV label position and text
        canvas.coords(self.label_id, x, y - 35)
        canvas.itemconfigure(
            self.label_id, text=f"UAV {self.uav_no} {self.state}", fill=fill_color
        )

    def __str__(self):
        """
        Returns a string representation of the UAV instance.
        """
        return (
            f"UAV {self.uav_no}:\n"
            f"  State: {self.state}\n"
            f"  Position: {self.pos}\n"
            f"  Direction: {self.direction}\n"
            f"  Target: {self.target.pos if self.target else 'None'}\n"
            f"  Leader: UAV {self.my_leader.uav_no if self.my_leader else 'None'}\n"
            f"  Slaves: {[slave.uav_no for slave in self.my_slave_list]}\n"
            f"  Ground Station: {self.ground.pos if self.ground else 'None'}"
        )

    def get_next_position(self, target_pos: Vector) -> Vector:
        """
        Calculates the next position towards the target without moving the UAV.
        """
        # Calculate the direction vector towards the target
        direction = Vector(target_pos.x - self.pos.x, target_pos.y - self.pos.y)

        # Normalize the direction vector to get unit vector
        distance = (direction.x**2 + direction.y**2) ** 0.5
        if distance == 0:
            return Vector(self.pos.x, self.pos.y)  # Already at the target

        unit_direction = Vector(direction.x / distance, direction.y / distance)

        # Calculate the next position
        next_x = self.pos.x + unit_direction.x * self.speed
        next_y = self.pos.y + unit_direction.y * self.speed

        return Vector(next_x, next_y)
