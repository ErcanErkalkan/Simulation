from typing import Optional

try:
    from djitellopy import Tello
except ImportError:  # pragma: no cover - depends on optional hardware SDK
    Tello = None

from simulation_app.domain.ground import Ground
from simulation_app.domain.uav import UAV
from simulation_app.domain.vector import Vector


class TelloDependencyError(RuntimeError):
    pass


class Tello_Drone(UAV):
    def __init__(
        self,
        pos: Optional[Vector] = None,
        direction: Optional[Vector] = None,
        uav_no: int = 1,
        ground: Optional[Ground] = None,
    ):
        super().__init__(pos, direction, uav_no, ground)
        self.drone = Tello() if Tello is not None else None
        self.connected = False

    def _require_driver(self) -> None:
        if self.drone is None:
            raise TelloDependencyError(
                "djitellopy is not installed. Tello control is unavailable."
            )

    def connect(self) -> None:
        if self.connected:
            return
        self._require_driver()
        print("[Tello] Connecting to Tello...")
        self.drone.connect()
        self.connected = True
        print("[Tello] Connected.")

    def disconnect(self) -> None:
        if not self.connected:
            return
        print("[Tello] Disconnecting drone.")
        self.drone.end()
        self.connected = False

    def takeoff(self) -> None:
        if self.connected:
            print("[Tello] Taking off.")
            self.drone.takeoff()
        else:
            print("[Tello] Not connected, cannot takeoff.")

    def land(self) -> None:
        if self.connected:
            print("[Tello] Landing.")
            self.drone.land()
        else:
            print("[Tello] Not connected, cannot land.")
