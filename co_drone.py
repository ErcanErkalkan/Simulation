# co_drone.py
import time
from codrone_edu.drone import Drone
from goal import Goal
from uav import UAV
from typing import Set, Optional, List
from vector import Vector
from ground import Ground


class co_Drone(UAV):
    def __init__(
        self,
        pos: Optional[Vector] = None,
        direction: Optional[Vector] = None,
        uav_no: int = 1,
        ground: Optional[Ground] = None,
        command_queue: Optional["queue.Queue"] = None,
    ):
        """
        Initializes a CoDrone UAV with additional drone-specific functionality.

        :param pos: Position of the UAV as a Vector.
        :param direction: Direction of the UAV as a Vector.
        :param uav_no: Unique identifier for the UAV.
        :param ground: Reference to the Ground object.
        """
        super().__init__(pos, direction, uav_no, ground)

        # Initialize the CoDrone-specific attributes
        self.drone = Drone()
        self.connected = False
        self.command_queue = command_queue  # <-- Added

    def connect(self, port: Optional[str] = None):
        """
        Connects the CoDrone to the specified port or automatically detects the port.

        :param port: Port name as a string (e.g., 'COM6'). If None, automatically detects the port.
        """
        if not self.connected:
            print("Connecting to CoDrone...")
            if port:
                self.drone.pair(port_name=port)
            else:
                self.drone.pair()
            self.connected = True
            print("Connected to CoDrone.")

    def disconnect(self):
        """
        Disconnects the CoDrone.
        """
        if self.connected:
            print("Disconnecting CoDrone...")
            self.drone.close()
            self.connected = False
            print("CoDrone disconnected.")

    def takeoff(self):
        """
        Commands the drone to take off.
        """
        if self.connected:
            print(f"Drone {self.uav_no} taking off...")
            self.drone.takeoff()
        else:
            print(f"Drone {self.uav_no} is not connected. Cannot take off.")

    def land(self):
        """
        Commands the drone to land.
        """
        if self.connected:
            print(f"Drone {self.uav_no} landing...")
            self.drone.land()
        else:
            print(f"Drone {self.uav_no} is not connected. Cannot land.")

    def move_to_real(self, target_pos: Vector, velocity: float = 0.5):
        """
        Moves the drone to the specified target position using absolute positioning.

        :param target_pos: The target position as a Vector.
        :param velocity: The velocity of the movement (default: 0.5 m/s).
        """
        if not self.connected:
            print(f"Drone {self.uav_no} is not connected. Cannot move.")
            return

        print(f"Drone {self.uav_no} moving to position: {target_pos}")
        target_x = target_pos.x
        target_y = target_pos.y
        target_z = 1  # Default height in meters (adjust as needed)
        print(target_x, target_y)
        self.drone.send_absolute_position(
            positionX=(target_x / 100),  # Convert cm to meters
            positionY=(target_y / 100),  # Convert cm to meters
            positionZ=target_z,         # Fixed height
            velocity=velocity,
            heading=0,
            rotationalVelocity=0,
        )

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
                # Here we only simulate the position by approaching in small steps
                step = direction_to_target.normalize()  # A unit direction
                # (You can add a speed multiplier if needed, e.g., step * 0.5)
                self.pos += step
                # Real Drone Command
                # If the real drone is connected, send physical movement commands
                if self.connected and self.command_queue:
                    # Add "MOVE_TO" command to the RealDroneThread queue
                    self.command_queue.put({
                        "type": "MOVE_TO",
                        "target_pos": step,  # Send the exact target
                        "velocity": 0.5
                    })

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
            # Simulate the position by approaching in small steps
            step = direction_to_target.normalize()  # A unit direction
            # (You can add a speed multiplier if needed, e.g., step * 0.5)
            self.pos += step
            # Real Drone Command
            if self.connected and self.command_queue:
                # Add "MOVE_TO" command to the RealDroneThread queue
                self.command_queue.put({
                    "type": "MOVE_TO",
                    "target_pos": step,
                    "velocity": 0.5
                })

    def move_to_position(self, target_pos: Vector):
        """
        Move the UAV towards a specified position.
        """
        next_pos = self.get_next_position(target_pos)
        self.pos = next_pos
        if self.connected and self.command_queue:
            # Add "MOVE_TO" command to the RealDroneThread queue
            self.command_queue.put({
                "type": "MOVE_TO",
                "target_pos": next_pos,
                "velocity": 0.5
            })

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
        if not self.my_leader:
            return

        direction_to_my_leader = self.my_leader.pos - self.pos
        distance = direction_to_my_leader.length()
        if distance < 1:  # If the UAV is close enough to the leader
            self.pos = self.my_leader.pos  # Snap to the leader's position
        else:
            # Simulate the position by approaching in small steps
            step = direction_to_my_leader.normalize()  # A unit direction
            self.pos += step
            # Real Drone Command
            if self.connected and self.command_queue:
                # Add "MOVE_TO" command to the RealDroneThread queue
                self.command_queue.put({
                    "type": "MOVE_TO",
                    "target_pos": step,
                    "velocity": 0.5
                })
