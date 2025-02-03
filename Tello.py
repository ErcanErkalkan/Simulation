import time
from djitellopy import Tello
from uav import UAV
from typing import Optional
from vector import Vector
from ground import Ground

class co_Drone(UAV):
    def __init__(
        self,
        pos: Optional[Vector] = None,
        direction: Optional[Vector] = None,
        uav_no: int = 1,
        ground: Optional[Ground] = None
    ):
        super().__init__(pos, direction, uav_no, ground)
        self.drone = Tello()
        self.connected = False

    def connect(self):
        """Connects to the Tello drone via WiFi."""
        if not self.connected:
            print("[Tello] Connecting to Tello...")
            self.drone.connect()
            self.connected = True
            print("[Tello] Connected.")
    
    def disconnect(self):
        """Disconnects from the Tello drone and turns off video stream if needed."""
        if self.connected:
            print("[Tello] Disconnecting drone.")
            self.drone.end()
            self.connected = False

    def takeoff(self):
        """Commands the Tello drone to take off."""
        if self.connected:
            print("[Tello] Taking off.")
            self.drone.takeoff()
        else:
            print("[Tello] Not connected, cannot takeoff.")

    def land(self):
        """Commands the Tello drone to land."""
        if self.connected:
            print("[Tello] Landing.")
            self.drone.land()
        else:
            print("[Tello] Not connected, cannot land.")
