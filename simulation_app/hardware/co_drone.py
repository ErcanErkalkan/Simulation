import time
from typing import Optional, TYPE_CHECKING

try:
    from codrone_edu.drone import Drone
except ImportError:  # pragma: no cover - depends on optional hardware SDK
    Drone = None

from simulation_app.domain.goal import Goal
from simulation_app.domain.ground import Ground
from simulation_app.domain.uav import UAV
from simulation_app.domain.vector import Vector

if TYPE_CHECKING:
    import queue


class CoDroneDependencyError(RuntimeError):
    pass


class co_Drone(UAV):
    def __init__(
        self,
        pos: Optional[Vector] = None,
        direction: Optional[Vector] = None,
        uav_no: int = 1,
        ground: Optional[Ground] = None,
        command_queue: Optional["queue.Queue"] = None,
    ):
        super().__init__(pos, direction, uav_no, ground)
        self.drone = Drone() if Drone is not None else None
        self.connected = False
        self.command_queue = command_queue

    def _require_driver(self) -> None:
        if self.drone is None:
            raise CoDroneDependencyError(
                "codrone_edu is not installed. Real drone control is unavailable."
            )

    def connect(self, port: Optional[str] = None) -> None:
        if self.connected:
            return
        self._require_driver()
        print("[co_Drone] Connecting...")
        if port:
            self.drone.pair(port_name=port)
        else:
            self.drone.pair()
        self.connected = True
        print("[co_Drone] Connected.")

    def disconnect(self) -> None:
        if not self.connected:
            return
        print("[co_Drone] Disconnecting drone.")
        self.drone.close()
        self.connected = False

    def takeoff(self) -> None:
        if self.connected:
            print("[co_Drone] Taking off.")
            self.drone.takeoff()
        else:
            print("[co_Drone] Not connected, cannot takeoff.")

    def land(self) -> None:
        if self.connected:
            print("[co_Drone] Landing.")
            self.drone.land()
        else:
            print(f"Drone {self.uav_no} is not connected. Cannot land.")

    def move_to_real(self, target_pos: Vector, velocity: float = 0.5) -> None:
        if not self.connected:
            print(f"Drone {self.uav_no} is not connected. Cannot move.")
            return

        self.drone.send_absolute_position(
            positionX=(target_pos.x / 100),
            positionY=(target_pos.y / 100),
            positionZ=1,
            velocity=velocity,
            heading=0,
            rotationalVelocity=0,
        )

    def move_to_target(self, target: Optional[Goal] = None) -> None:
        if target is not None:
            self.target = target
        if self.target is None:
            return

        next_pos = self.get_next_position(self.target.pos)
        reached_target = next_pos == self.target.pos
        self._move_to_simulated_position(next_pos)

        if not reached_target:
            return

        was_ground_leader = self.state == "Ground_Leader"
        self.state = "Free"
        self.target.state = "Visited"
        self.target.last_visited_time = time.time()
        self.target = None
        if was_ground_leader:
            self.delete_my_relay_list()
        self.delete_my_slave_list()

    def move(self, target_pos: Vector) -> None:
        self._move_to_simulated_position(self.get_next_position(target_pos))

    def move_to_position(self, target_pos: Vector) -> None:
        self._move_to_simulated_position(self.get_next_position(target_pos))

    def move_to_ground(self) -> None:
        if self.ground is not None:
            self.move(self.ground.pos)

    def move_to_leader(self) -> None:
        if self.my_leader is None:
            return
        self._move_to_simulated_position(self.get_next_position(self.my_leader.pos))

    def _move_to_simulated_position(self, next_pos: Vector) -> None:
        if next_pos == self.pos:
            return
        self.pos = next_pos
        if self.connected and self.command_queue:
            self.command_queue.put(
                {
                    "type": "MOVE_TO",
                    "target_pos": next_pos,
                    "velocity": 0.5,
                }
            )
