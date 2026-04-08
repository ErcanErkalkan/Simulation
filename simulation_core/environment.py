from __future__ import annotations

from typing import Dict, List, Optional, Type

from simulation_app.domain.goal import Goal
from simulation_app.domain.ground import Ground
from simulation_app.domain.uav import UAV
from simulation_app.domain.vector import Vector
from simulation_app.support.generate_goal import GenerateGoal
from simulation_app.support.generate_uav import GenerateUAV


class SimulationEnvironment:
    def __init__(self, comm_thr: float = 200):
        self.comm_thr = float(comm_thr)
        self.goals: List[Goal] = []
        self.uavs: List[UAV] = []
        self.ground: Optional[Ground] = None

    def set_comm_thr(self, value: float) -> None:
        if value <= 0:
            raise ValueError("Communication threshold must be positive.")
        self.comm_thr = float(value)

    def add_goal(self, x: int, y: int) -> Goal:
        goal = Goal(pos=Vector(x, y), goal_no=len(self.goals) + 1)
        self.goals.append(goal)
        return goal

    def add_uav(self, x: int, y: int, uav_cls: Type[UAV] = UAV, **kwargs) -> UAV:
        uav = uav_cls(
            pos=Vector(x, y),
            uav_no=len(self.uavs) + 1,
            ground=self.ground,
            **kwargs,
        )
        self.uavs.append(uav)
        return uav

    def add_existing_uav(self, uav: UAV) -> UAV:
        uav.uav_no = len(self.uavs) + 1
        uav.ground = self.ground
        self.uavs.append(uav)
        return uav

    def generate_ground(self, x: int, y: int) -> Ground:
        self.ground = Ground(pos=Vector(x, y), color="lightgreen")
        self._assign_ground_to_uavs()
        return self.ground

    def generate_goals(
        self, goal_count: int, canvas_width: int, canvas_height: int
    ) -> List[Goal]:
        self.goals = GenerateGoal.run(goal_count, canvas_width, canvas_height)
        return self.goals

    def generate_uavs(
        self, uav_count: int, canvas_width: int, canvas_height: int
    ) -> List[UAV]:
        self.uavs = GenerateUAV.run(
            count=uav_count,
            comm_thr=self.comm_thr,
            ground=self.ground,
            canvas_width=canvas_width,
            canvas_height=canvas_height,
        )
        self._renumber_uavs()
        return self.uavs

    def update_goal_states(self, current_time: float, threshold1: float) -> None:
        for goal in self.goals:
            goal.update_state(current_time, threshold1)

    def all_goals_visited(self) -> bool:
        return bool(self.goals) and all(goal.state == "Visited" for goal in self.goals)

    def reset(self) -> None:
        self.goals = []
        self.uavs = []
        self.ground = None

    def to_dict(self) -> Dict[str, object]:
        return {
            "Ground": (
                {"X": self.ground.pos.x, "Y": self.ground.pos.y}
                if self.ground
                else None
            ),
            "UAVs": [
                {"X": uav.pos.x, "Y": uav.pos.y, "Type": uav.__class__.__name__}
                for uav in self.uavs
            ],
            "Goals": [
                {
                    "X": goal.pos.x,
                    "Y": goal.pos.y,
                    "State": self._persisted_goal_state(goal.state),
                    "LastVisitedTime": goal.last_visited_time,
                }
                for goal in self.goals
            ],
        }

    def load_from_dict(
        self,
        data: Dict[str, object],
        uav_types: Optional[Dict[str, Type[UAV]]] = None,
    ) -> None:
        self.reset()
        uav_types = uav_types or {"UAV": UAV}

        ground_data = data.get("Ground")
        if isinstance(ground_data, dict):
            self.generate_ground(int(ground_data["X"]), int(ground_data["Y"]))

        for index, uav_data in enumerate(data.get("UAVs", []), start=1):
            if not isinstance(uav_data, dict):
                continue
            uav_type_name = uav_data.get("Type", "UAV")
            uav_cls = uav_types.get(str(uav_type_name), UAV)
            uav = uav_cls(
                pos=Vector(uav_data["X"], uav_data["Y"]),
                uav_no=index,
                ground=self.ground,
            )
            self.uavs.append(uav)

        for index, goal_data in enumerate(data.get("Goals", []), start=1):
            if not isinstance(goal_data, dict):
                continue
            goal = Goal(
                pos=Vector(goal_data["X"], goal_data["Y"]),
                goal_no=index,
                state=self._persisted_goal_state(goal_data.get("State", "Free")),
            )
            goal.last_visited_time = goal_data.get("LastVisitedTime")
            self.goals.append(goal)

        self._renumber_uavs()
        self._assign_ground_to_uavs()

    def _renumber_uavs(self) -> None:
        for index, uav in enumerate(self.uavs, start=1):
            uav.uav_no = index
            uav.ground = self.ground

    def _assign_ground_to_uavs(self) -> None:
        for uav in self.uavs:
            uav.ground = self.ground

    @staticmethod
    def _persisted_goal_state(state: object) -> str:
        if state == "Visited":
            return "Visited"
        return "Free"
