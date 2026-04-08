from __future__ import annotations

import time
from typing import Callable, Dict, List, Optional, Type

from simulation_app.domain.goal import Goal
from simulation_app.domain.ground import Ground
from simulation_app.domain.uav import UAV
from simulation_app.support.distance import Distance
from simulation_core.algorithm_api import (
    AlgorithmPlan,
    GoalAssignment,
    GoalView,
    GroundView,
    Position,
    RelayAssignment,
    SimulationContext,
    SlaveAssignment,
    UavView,
)
from simulation_core.config import SimulationConfig
from simulation_core.environment import SimulationEnvironment
from simulation_core.registry import AlgorithmRegistry


class SimulationEngine:
    def __init__(self, comm_thr: float = 200, algorithm_name: Optional[str] = None):
        self.environment = SimulationEnvironment(comm_thr=comm_thr)
        self.algorithm_registry = AlgorithmRegistry()
        self.algorithm = self.algorithm_registry.create(
            algorithm_name or self.algorithm_registry.default_name()
        )
        self.config_provider: Callable[[], SimulationConfig] = lambda: SimulationConfig()
        self.simulation_running = False
        self.start_time: Optional[float] = None
        self.uav_types: Dict[str, Type[UAV]] = {"UAV": UAV}

    @property
    def goals(self) -> List[Goal]:
        return self.environment.goals

    @goals.setter
    def goals(self, value: List[Goal]) -> None:
        self.environment.goals = value

    @property
    def uavs(self) -> List[UAV]:
        return self.environment.uavs

    @uavs.setter
    def uavs(self, value: List[UAV]) -> None:
        self.environment.uavs = value

    @property
    def ground(self) -> Optional[Ground]:
        return self.environment.ground

    @ground.setter
    def ground(self, value: Optional[Ground]) -> None:
        self.environment.ground = value

    @property
    def comm_thr(self) -> float:
        return self.environment.comm_thr

    @comm_thr.setter
    def comm_thr(self, value: float) -> None:
        self.environment.set_comm_thr(value)

    def register_uav_type(self, uav_cls: Type[UAV]) -> None:
        self.uav_types[uav_cls.__name__] = uav_cls

    def set_config_provider(self, provider: Callable[[], SimulationConfig]) -> None:
        self.config_provider = provider

    def get_runtime_config(self) -> SimulationConfig:
        config = self.config_provider()
        if isinstance(config, dict):
            config = SimulationConfig(**config)
        if not isinstance(config, SimulationConfig):
            raise TypeError("Config provider must return SimulationConfig or dict.")
        config.validate()
        return config

    def get_algorithm_metadata(self):
        return self.algorithm_registry.metadata()

    def set_algorithm(self, algorithm_name: str) -> None:
        self.algorithm = self.algorithm_registry.create(algorithm_name)
        self.algorithm.reset()

    def get_current_algorithm_name(self) -> str:
        return self.algorithm.name

    def add_goal(self, x: int, y: int) -> Goal:
        return self.environment.add_goal(x, y)

    def generate_ground(self, x: int, y: int) -> Ground:
        return self.environment.generate_ground(x, y)

    def generate_goals(self, goal_count: int, canvas_width: int, canvas_height: int):
        return self.environment.generate_goals(goal_count, canvas_width, canvas_height)

    def generate_uavs(self, uav_count: int, canvas_width: int, canvas_height: int):
        return self.environment.generate_uavs(uav_count, canvas_width, canvas_height)

    def add_uav(self, x: int, y: int, uav_cls: Type[UAV] = UAV, **kwargs) -> UAV:
        return self.environment.add_uav(x, y, uav_cls=uav_cls, **kwargs)

    def add_existing_uav(self, uav: UAV) -> UAV:
        return self.environment.add_existing_uav(uav)

    def reset_simulation(self) -> None:
        self.stop_simulation()
        self.environment.reset()
        self.start_time = None
        self.algorithm.reset()

    def start_simulation(self) -> None:
        config = self.get_runtime_config()
        self._apply_runtime_config(config)
        self.simulation_running = True
        self.start_time = time.time()
        self.algorithm.reset()

    def stop_simulation(self) -> None:
        self.simulation_running = False

    def move_uavs(self) -> Optional[str]:
        if not self.simulation_running:
            return None

        config = self.get_runtime_config()
        self._apply_runtime_config(config)
        current_time = time.time()

        if config.target_eval_mode == "revisit" and self.start_time is not None:
            elapsed = current_time - self.start_time
            if elapsed >= config.simulation_time:
                self.stop_simulation()
                return "time_elapsed"
            self.environment.update_goal_states(current_time, config.threshold1)

        context = self._build_context(current_time, config)
        plan = self.algorithm.build_plan(context)
        self._apply_plan(plan)
        self._advance_uavs(current_time, config)

        if (
            config.target_eval_mode == "single_visit"
            and self.environment.all_goals_visited()
        ):
            self.stop_simulation()
            return "all_goals_visited"

        return None

    def to_dict(self) -> Dict[str, object]:
        return self.environment.to_dict()

    def load_from_dict(self, data: Dict[str, object]) -> None:
        self.stop_simulation()
        self.environment.load_from_dict(data, self.uav_types)
        self.algorithm.reset()

    def _build_context(
        self, current_time: float, config: SimulationConfig
    ) -> SimulationContext:
        ground = None
        if self.environment.ground is not None:
            ground = GroundView(position=Position.from_vector(self.environment.ground.pos))

        uavs = tuple(
            UavView(
                id=uav.uav_no,
                position=Position.from_vector(uav.pos),
                state=uav.state,
                uav_type=uav.__class__.__name__,
                speed=float(getattr(uav, "speed", config.uav_speed)),
                target_goal_id=uav.target.goal_no if uav.target else None,
                leader_id=uav.my_leader.uav_no if uav.my_leader else None,
                relay_target=(
                    Position.from_vector(uav.target_position)
                    if hasattr(uav, "target_position")
                    else None
                ),
            )
            for uav in self.environment.uavs
        )

        goals = tuple(
            GoalView(
                id=goal.goal_no,
                position=Position.from_vector(goal.pos),
                state=goal.state,
                last_visited_time=goal.last_visited_time,
            )
            for goal in self.environment.goals
        )

        return SimulationContext(
            current_time=current_time,
            comm_thr=self.environment.comm_thr,
            config=config,
            uavs=uavs,
            goals=goals,
            ground=ground,
        )

    def _apply_runtime_config(self, config: SimulationConfig) -> None:
        for uav in self.environment.uavs:
            uav.speed = float(config.uav_speed)

    def _apply_plan(self, plan: AlgorithmPlan) -> None:
        if plan.clear_uav_roles:
            for uav in self.environment.uavs:
                self._free_uav(uav)
        if plan.clear_goal_assignments:
            for goal in self.environment.goals:
                self._free_goal(goal)

        for uav_id in plan.free_uav_ids:
            uav = self._get_uav(uav_id)
            if uav is not None:
                self._free_uav(uav)

        for goal_id in plan.free_goal_ids:
            goal = self._get_goal(goal_id)
            if goal is not None:
                self._free_goal(goal)

        for assignment in plan.goal_assignments:
            self._apply_goal_assignment(assignment)

        for assignment in plan.relay_assignments:
            self._apply_relay_assignment(assignment)

        for assignment in plan.slave_assignments:
            self._apply_slave_assignment(assignment)

    def _apply_goal_assignment(self, assignment: GoalAssignment) -> None:
        uav = self._get_uav(assignment.uav_id)
        goal = self._get_goal(assignment.goal_id)
        if uav is None or goal is None:
            return
        self._free_uav(uav)
        self._free_goal(goal)
        uav.state = assignment.role
        uav.target = goal
        goal.state = "Assigned"

    def _apply_relay_assignment(self, assignment: RelayAssignment) -> None:
        uav = self._get_uav(assignment.uav_id)
        leader = self._get_uav(assignment.leader_id)
        if uav is None or leader is None or uav is leader:
            return
        self._free_uav(uav)
        uav.state = "Relay"
        uav.my_leader = leader
        uav.target_position = assignment.target_position.to_vector()
        leader.my_slave_list.add(uav)

    def _apply_slave_assignment(self, assignment: SlaveAssignment) -> None:
        uav = self._get_uav(assignment.uav_id)
        leader = self._get_uav(assignment.leader_id)
        if uav is None or leader is None or uav is leader:
            return
        self._free_uav(uav)
        uav.state = "Slave"
        uav.my_leader = leader
        leader.my_slave_list.add(uav)

    def _advance_uavs(self, current_time: float, config: SimulationConfig) -> None:
        for uav in self.environment.uavs:
            context = self._build_context(current_time, config)
            uav_view = context.uav_by_id(uav.uav_no)
            if uav_view is None:
                continue

            if uav.state in {"Leader", "Ground_Leader"} and uav.target is not None:
                target_position = Position.from_vector(uav.target.pos)
                if self.algorithm.allow_movement(context, uav_view, target_position):
                    uav.move_to_target()
            elif uav.state == "Relay" and hasattr(uav, "target_position"):
                target_position = Position.from_vector(uav.target_position)
                if self.algorithm.allow_movement(context, uav_view, target_position):
                    uav.move_to_position(uav.target_position)
                elif uav.my_leader is not None:
                    leader_position = Position.from_vector(uav.my_leader.pos)
                    if self.algorithm.allow_movement(context, uav_view, leader_position):
                        uav.move_to_leader()
            elif uav.state == "Slave" and uav.my_leader is not None:
                if Distance.distance_between(uav, uav.my_leader) > self.environment.comm_thr * 0.8:
                    leader_position = Position.from_vector(uav.my_leader.pos)
                    if self.algorithm.allow_movement(context, uav_view, leader_position):
                        uav.move_to_leader()

    def _free_uav(self, uav: UAV) -> None:
        uav.delete_my_slave_list()
        uav.state = "Free"
        uav.my_leader = None
        if uav.target is not None:
            self._free_goal(uav.target)
            uav.target = None
        if hasattr(uav, "target_position"):
            delattr(uav, "target_position")

    def _free_goal(self, goal: Goal) -> None:
        if goal.state != "Visited":
            goal.state = "Free"

    def _get_uav(self, uav_id: int) -> Optional[UAV]:
        return next((uav for uav in self.environment.uavs if uav.uav_no == uav_id), None)

    def _get_goal(self, goal_id: int) -> Optional[Goal]:
        return next((goal for goal in self.environment.goals if goal.goal_no == goal_id), None)
