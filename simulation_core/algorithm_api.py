from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple

from simulation_core.config import SimulationConfig


@dataclass(frozen=True)
class Position:
    x: float
    y: float

    @classmethod
    def from_vector(cls, value) -> "Position":
        return cls(float(value.x), float(value.y))

    def to_vector(self):
        from simulation_app.domain.vector import Vector

        return Vector(self.x, self.y)


@dataclass(frozen=True)
class GoalView:
    id: int
    position: Position
    state: str
    last_visited_time: Optional[float]


@dataclass(frozen=True)
class UavView:
    id: int
    position: Position
    state: str
    uav_type: str
    target_goal_id: Optional[int]
    leader_id: Optional[int]
    relay_target: Optional[Position]


@dataclass(frozen=True)
class GroundView:
    position: Position


@dataclass(frozen=True)
class GoalAssignment:
    uav_id: int
    goal_id: int
    role: str = "Leader"


@dataclass(frozen=True)
class RelayAssignment:
    uav_id: int
    leader_id: int
    target_position: Position


@dataclass(frozen=True)
class SlaveAssignment:
    uav_id: int
    leader_id: int


@dataclass(frozen=True)
class AlgorithmPlan:
    clear_uav_roles: bool = False
    clear_goal_assignments: bool = False
    goal_assignments: Tuple[GoalAssignment, ...] = ()
    relay_assignments: Tuple[RelayAssignment, ...] = ()
    slave_assignments: Tuple[SlaveAssignment, ...] = ()
    free_uav_ids: Tuple[int, ...] = ()
    free_goal_ids: Tuple[int, ...] = ()

    @classmethod
    def empty(cls) -> "AlgorithmPlan":
        return cls()


@dataclass(frozen=True)
class SimulationContext:
    current_time: float
    comm_thr: float
    config: SimulationConfig
    uavs: Tuple[UavView, ...]
    goals: Tuple[GoalView, ...]
    ground: Optional[GroundView]

    def uav_by_id(self, uav_id: int) -> Optional[UavView]:
        return next((uav for uav in self.uavs if uav.id == uav_id), None)

    def goal_by_id(self, goal_id: int) -> Optional[GoalView]:
        return next((goal for goal in self.goals if goal.id == goal_id), None)

    def free_uavs(self) -> Tuple[UavView, ...]:
        return tuple(uav for uav in self.uavs if uav.state == "Free")

    def free_goals(self) -> Tuple[GoalView, ...]:
        return tuple(goal for goal in self.goals if goal.state == "Free")


class SimulationAlgorithm(ABC):
    name = "base"
    display_name = "Base"
    description = ""

    def reset(self) -> None:
        pass

    @abstractmethod
    def build_plan(self, context: SimulationContext) -> AlgorithmPlan:
        raise NotImplementedError

    def allow_movement(
        self,
        context: SimulationContext,
        uav: UavView,
        target_position: Position,
    ) -> bool:
        return True
