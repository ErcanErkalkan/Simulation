# co_drone.py
import time
from codrone_edu.drone import Drone
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
        self.drone = Drone()
        self.connected = False
    
    def connect(self, port=None):
        if not self.connected:
            print("[co_Drone] Connecting...")
            if port:
                self.drone.pair(port_name=port)
            else:
                self.drone.pair()
            self.connected = True
            print("[co_Drone] Connected.")

    def disconnect(self):
        if self.connected:
            print("[co_Drone] Disconnecting drone.")
            self.drone.close()
            self.connected = False

    def takeoff(self):
        if self.connected:
            print("[co_Drone] Taking off.")
            self.drone.takeoff()
        else:
            print("[co_Drone] Not connected, cannot takeoff.")

    def land(self):
        if self.connected:
            print("[co_Drone] Landing.")
            self.drone.land()
        else:
            print("[co_Drone] Not connected, cannot land.")
