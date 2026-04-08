from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set

import numpy as np

from algorithms.base import (
    AlgorithmPlan,
    GoalAssignment,
    Position,
    RelayAssignment,
    SimulationAlgorithm,
    SimulationContext,
    SlaveAssignment,
    UavView,
)
from simulation_core.topology import (
    build_distance_matrix,
    build_uav_goal_distance_matrix,
    distance_between,
    find_connected_components,
    find_risky_links,
)
from simulation_app.support.valuation import Valuation


@dataclass
class GoalState:
    id: int
    position: Position
    state: str
    last_visited_time: Optional[float]


@dataclass
class UavState:
    id: int
    position: Position
    state: str = "Free"
    target_goal_id: Optional[int] = None
    leader_id: Optional[int] = None
    relay_target: Optional[Position] = None
    slave_ids: Set[int] = field(default_factory=set)


class ConnectivityAwareAlgorithm(SimulationAlgorithm):
    name = "connectivity_current"
    display_name = "Current Connectivity"
    description = "Baglanti korumali lider/slave/relay mantigini calistirir."

    def build_plan(self, context: SimulationContext) -> AlgorithmPlan:
        uavs, goals = self.build_state(context)
        self.assign_uavs_to_goals(context, uavs, goals)

        positions = [uav.position for uav in uavs.values()]
        if context.ground is not None:
            positions.append(context.ground.position)
        adj_matrix = build_distance_matrix(positions)
        components = find_connected_components(adj_matrix, context.comm_thr)
        if len(components) > 1:
            self.assign_relay(context, uavs, goals)

        goal_assignments = []
        relay_assignments = []
        slave_assignments = []

        for uav in uavs.values():
            if uav.state in {"Leader", "Ground_Leader"} and uav.target_goal_id is not None:
                goal_assignments.append(
                    GoalAssignment(
                        uav_id=uav.id,
                        goal_id=uav.target_goal_id,
                        role=uav.state,
                    )
                )
            elif uav.state == "Relay" and uav.leader_id is not None and uav.relay_target is not None:
                relay_assignments.append(
                    RelayAssignment(
                        uav_id=uav.id,
                        leader_id=uav.leader_id,
                        target_position=uav.relay_target,
                    )
                )
            elif uav.state == "Slave" and uav.leader_id is not None:
                slave_assignments.append(
                    SlaveAssignment(uav_id=uav.id, leader_id=uav.leader_id)
                )

        return AlgorithmPlan(
            clear_uav_roles=True,
            clear_goal_assignments=True,
            goal_assignments=tuple(goal_assignments),
            relay_assignments=tuple(relay_assignments),
            slave_assignments=tuple(slave_assignments),
        )

    def allow_movement(
        self,
        context: SimulationContext,
        uav: UavView,
        target_position: Position,
    ) -> bool:
        next_position = self.get_next_position(
            uav.position,
            target_position,
            uav.speed,
        )
        positions = [
            next_position if candidate.id == uav.id else candidate.position
            for candidate in context.uavs
        ]
        if context.ground is not None:
            positions.append(context.ground.position)
        adj_matrix = build_distance_matrix(positions)
        components = find_connected_components(adj_matrix, context.comm_thr * 1.1)
        return len(components) <= 1

    def build_state(
        self, context: SimulationContext
    ) -> tuple[Dict[int, UavState], Dict[int, GoalState]]:
        goals = {
            goal.id: GoalState(
                id=goal.id,
                position=goal.position,
                state="Visited" if goal.state == "Visited" else "Free",
                last_visited_time=goal.last_visited_time,
            )
            for goal in context.goals
        }
        uavs = {
            uav.id: UavState(id=uav.id, position=uav.position)
            for uav in context.uavs
        }

        for uav in context.uavs:
            if uav.state not in {"Leader", "Ground_Leader"} or uav.target_goal_id is None:
                continue
            goal = goals.get(uav.target_goal_id)
            if goal is None or goal.state == "Visited":
                continue
            uavs[uav.id].state = uav.state
            uavs[uav.id].target_goal_id = goal.id
            goal.state = "Assigned"

        return uavs, goals

    def assign_uavs_to_goals(
        self,
        context: SimulationContext,
        uavs: Dict[int, UavState],
        goals: Dict[int, GoalState],
    ) -> bool:
        free_uavs = [uav for uav in uavs.values() if uav.state == "Free"]
        free_goals = [goal for goal in goals.values() if goal.state == "Free"]
        if not free_uavs or not free_goals:
            return False

        if context.config.target_eval_mode == "revisit":
            valuations = []
            for uav in free_uavs:
                for goal in free_goals:
                    valuations.append(
                        (
                            Valuation.composite_valuation(
                                self.get_time_since_last_visit(goal, context),
                                uav.position.to_vector(),
                                goal.position.to_vector(),
                                context.config.threshold1,
                                context.config.threshold2,
                            ),
                            uav.id,
                            goal.id,
                        )
                    )

            valuations.sort(reverse=True, key=lambda item: item[0])
            assigned_uavs = set()
            assigned_goals = set()
            changed = False
            for _, uav_id, goal_id in valuations:
                if uav_id in assigned_uavs or goal_id in assigned_goals:
                    continue
                self.become_leader(uavs, goals, uav_id, goal_id)
                assigned_uavs.add(uav_id)
                assigned_goals.add(goal_id)
                changed = True
                if len(assigned_uavs) >= len(free_uavs):
                    break
            return changed

        distance_matrix = build_uav_goal_distance_matrix(
            [uav.position for uav in free_uavs],
            [goal.position for goal in free_goals],
        )
        assigned_uav_indices = set()
        assigned_goal_indices = set()
        changed = False

        while (
            len(assigned_uav_indices) < len(free_uavs)
            and len(assigned_goal_indices) < len(free_goals)
        ):
            uav_index, goal_index = np.unravel_index(
                np.argmin(distance_matrix, axis=None), distance_matrix.shape
            )
            if np.isinf(distance_matrix[uav_index, goal_index]):
                break
            self.become_leader(
                uavs,
                goals,
                free_uavs[uav_index].id,
                free_goals[goal_index].id,
            )
            assigned_uav_indices.add(uav_index)
            assigned_goal_indices.add(goal_index)
            distance_matrix[uav_index, :] = float("inf")
            distance_matrix[:, goal_index] = float("inf")
            changed = True

        return changed

    def assign_relay(
        self,
        context: SimulationContext,
        uavs: Dict[int, UavState],
        goals: Dict[int, GoalState],
    ) -> None:
        uav_order = list(uavs)
        positions = [uavs[uav_id].position for uav_id in uav_order]
        if context.ground is not None:
            positions.append(context.ground.position)
        adj_matrix = build_distance_matrix(positions)
        components = find_connected_components(adj_matrix, context.comm_thr)
        risky_links = find_risky_links(components, adj_matrix, context.comm_thr)
        ground_index = len(uav_order)

        for link in risky_links:
            involves_ground = link[0] == ground_index or link[1] == ground_index
            if not involves_ground or context.ground is None:
                continue

            self.reset_uavs(uavs, goals)
            self.reset_goals(goals)
            leader = self.assign_most_profitable_uav(context, uavs, goals)
            if leader is None or leader.target_goal_id is None:
                return
            goal = goals[leader.target_goal_id]
            relay_count = self.calculate_number_of_relays(
                context,
                goal.position,
                context.ground.position,
            )
            self.allocate_relay_uavs_line(
                context,
                uavs,
                leader.id,
                relay_count,
            )
            break

        for link in risky_links:
            if link[0] == ground_index or link[1] == ground_index:
                continue
            uav_a = uavs[uav_order[link[0]]]
            uav_b = uavs[uav_order[link[1]]]
            priority_a = self.get_priority(uav_a)
            priority_b = self.get_priority(uav_b)

            if priority_a != priority_b:
                if priority_a > priority_b:
                    self.make_slave(uavs, goals, uav_b.id, uav_a.id)
                else:
                    self.make_slave(uavs, goals, uav_a.id, uav_b.id)
            elif priority_a == priority_b == 2:
                distance_a = distance_between(uav_a.position, uav_a.relay_target)
                distance_b = distance_between(uav_b.position, uav_b.relay_target)
                if distance_a < distance_b:
                    self.make_slave(uavs, goals, uav_b.id, uav_a.id)
                else:
                    self.make_slave(uavs, goals, uav_a.id, uav_b.id)
            else:
                self.resolve_same_priority_link(context, uavs, goals, uav_a.id, uav_b.id)

        self.assign_uavs_to_goals(context, uavs, goals)

    def resolve_same_priority_link(
        self,
        context: SimulationContext,
        uavs: Dict[int, UavState],
        goals: Dict[int, GoalState],
        uav_a_id: int,
        uav_b_id: int,
    ) -> None:
        uav_a = uavs[uav_a_id]
        uav_b = uavs[uav_b_id]

        if uav_a.state == "Leader" and uav_b.state == "Leader":
            self.resolve_leader_conflict(uavs, goals, uav_a_id, uav_b_id)
        elif uav_a.state == "Free" and uav_b.state == "Free":
            leader = self.get_closest_leader(uavs, uav_a_id)
            if leader is not None:
                self.make_slave(uavs, goals, uav_a_id, leader.id)
                self.make_slave(uavs, goals, uav_b_id, leader.id)
        elif (uav_a.state == "Slave" and uav_b.state == "Free") or (
            uav_a.state == "Free" and uav_b.state == "Slave"
        ):
            leader_id = uav_a.leader_id if uav_a.state == "Slave" else uav_b.leader_id
            slave_id = uav_b.id if uav_a.state == "Slave" else uav_a.id
            if leader_id is not None:
                self.make_slave(uavs, goals, slave_id, leader_id)
        elif (uav_a.state == "Leader" and uav_b.state == "Free") or (
            uav_a.state == "Free" and uav_b.state == "Leader"
        ):
            leader_id = uav_a.id if uav_a.state == "Leader" else uav_b.id
            slave_id = uav_b.id if uav_a.state == "Leader" else uav_a.id
            self.make_slave(uavs, goals, slave_id, leader_id)
        elif uav_a.state == "Leader" and uav_b.state == "Slave":
            if uav_b.leader_id is not None and uav_b.leader_id != uav_a.id:
                self.resolve_leader_conflict(uavs, goals, uav_a.id, uav_b.leader_id)
        elif uav_a.state == "Slave" and uav_b.state == "Leader":
            if uav_a.leader_id is not None and uav_a.leader_id != uav_b.id:
                self.resolve_leader_conflict(uavs, goals, uav_a.leader_id, uav_b.id)
        elif uav_a.state == "Slave" and uav_b.state == "Slave":
            if (
                uav_a.leader_id is not None
                and uav_b.leader_id is not None
                and uav_a.leader_id != uav_b.leader_id
            ):
                self.resolve_leader_conflict(uavs, goals, uav_a.leader_id, uav_b.leader_id)

    def become_leader(
        self,
        uavs: Dict[int, UavState],
        goals: Dict[int, GoalState],
        uav_id: int,
        goal_id: int,
        role: str = "Leader",
    ) -> bool:
        uav = uavs[uav_id]
        goal = goals[goal_id]
        if self.get_priority(uav) > 1 or uav.state != "Free":
            return False
        uav.state = role
        uav.target_goal_id = goal_id
        goal.state = "Assigned"
        return True

    def reset_uavs(
        self, uavs: Dict[int, UavState], goals: Dict[int, GoalState]
    ) -> None:
        for uav in uavs.values():
            self.free_uav(uavs, goals, uav.id)

    def reset_goals(self, goals: Dict[int, GoalState]) -> None:
        for goal in goals.values():
            if goal.state != "Visited":
                goal.state = "Free"

    def free_uav(
        self,
        uavs: Dict[int, UavState],
        goals: Dict[int, GoalState],
        uav_id: int,
    ) -> None:
        uav = uavs[uav_id]
        if uav.leader_id is not None:
            leader = uavs.get(uav.leader_id)
            if leader is not None:
                leader.slave_ids.discard(uav.id)
            uav.leader_id = None

        for slave_id in list(uav.slave_ids):
            slave = uavs[slave_id]
            slave.state = "Free"
            slave.leader_id = None
            slave.relay_target = None
            uav.slave_ids.discard(slave_id)

        if uav.target_goal_id is not None:
            goal = goals[uav.target_goal_id]
            if goal.state != "Visited":
                goal.state = "Free"
            uav.target_goal_id = None

        uav.state = "Free"
        uav.relay_target = None

    def get_priority(self, uav: UavState) -> int:
        if uav.state == "Ground_Leader":
            return 3
        if uav.state == "Relay":
            return 2
        if uav.state in {"Leader", "Slave", "Free"}:
            return 1
        return 0

    def make_slave(
        self,
        uavs: Dict[int, UavState],
        goals: Dict[int, GoalState],
        slave_id: int,
        leader_id: int,
    ) -> None:
        if slave_id == leader_id:
            return
        slave = uavs[slave_id]
        leader = uavs[leader_id]

        if slave.target_goal_id is not None:
            goal = goals[slave.target_goal_id]
            if goal.state != "Visited":
                goal.state = "Free"
            slave.target_goal_id = None

        if slave.leader_id is not None:
            previous_leader = uavs.get(slave.leader_id)
            if previous_leader is not None:
                previous_leader.slave_ids.discard(slave.id)

        for nested_slave_id in list(slave.slave_ids):
            nested_slave = uavs[nested_slave_id]
            nested_slave.state = "Slave"
            nested_slave.leader_id = leader.id
            leader.slave_ids.add(nested_slave_id)
        slave.slave_ids.clear()

        slave.state = "Slave"
        slave.leader_id = leader.id
        slave.relay_target = None
        leader.slave_ids.add(slave.id)

    def resolve_leader_conflict(
        self,
        uavs: Dict[int, UavState],
        goals: Dict[int, GoalState],
        uav_a_id: int,
        uav_b_id: int,
    ) -> None:
        uav_a = uavs[uav_a_id]
        uav_b = uavs[uav_b_id]
        distance_a = (
            distance_between(uav_a.position, goals[uav_a.target_goal_id].position)
            if uav_a.target_goal_id is not None
            else float("inf")
        )
        distance_b = (
            distance_between(uav_b.position, goals[uav_b.target_goal_id].position)
            if uav_b.target_goal_id is not None
            else float("inf")
        )
        if distance_a > distance_b:
            self.make_slave(uavs, goals, uav_a_id, uav_b_id)
        else:
            self.make_slave(uavs, goals, uav_b_id, uav_a_id)

    def get_closest_leader(
        self, uavs: Dict[int, UavState], uav_id: int
    ) -> Optional[UavState]:
        candidates = [
            candidate
            for candidate in uavs.values()
            if candidate.state in {"Ground_Leader", "Relay", "Leader"}
        ]
        if not candidates:
            return None
        current = uavs[uav_id]
        return min(
            candidates,
            key=lambda leader: distance_between(leader.position, current.position),
        )

    def assign_most_profitable_uav(
        self,
        context: SimulationContext,
        uavs: Dict[int, UavState],
        goals: Dict[int, GoalState],
    ) -> Optional[UavState]:
        free_uavs = [uav for uav in uavs.values() if uav.state == "Free"]
        available_goals = [goal for goal in goals.values() if goal.state == "Free"]
        if not free_uavs or not available_goals:
            return None

        valuations = []
        if context.config.target_eval_mode == "revisit":
            for uav in free_uavs:
                for goal in available_goals:
                    valuations.append(
                        (
                            Valuation.composite_valuation(
                                self.get_time_since_last_visit(goal, context),
                                uav.position.to_vector(),
                                goal.position.to_vector(),
                                context.config.threshold1,
                                context.config.threshold2,
                            ),
                            uav.id,
                            goal.id,
                        )
                    )
        else:
            for uav in free_uavs:
                for goal in available_goals:
                    distance = distance_between(uav.position, goal.position)
                    valuation = 1 / distance if distance != 0 else float("inf")
                    valuations.append((valuation, uav.id, goal.id))

        if not valuations:
            return None

        _, uav_id, goal_id = max(valuations, key=lambda item: item[0])
        self.become_leader(uavs, goals, uav_id, goal_id, role="Ground_Leader")
        return uavs[uav_id]

    def get_time_since_last_visit(
        self,
        goal: GoalState,
        context: SimulationContext,
    ) -> float:
        if goal.last_visited_time is None:
            return context.config.threshold2 + 1
        return max(context.current_time - goal.last_visited_time, 0.0)

    def calculate_number_of_relays(
        self,
        context: SimulationContext,
        uav_position: Position,
        station_position: Position,
    ) -> int:
        distance = distance_between(uav_position, station_position)
        return max(int(np.ceil(distance / (0.9 * context.comm_thr))) - 1, 0)

    def allocate_relay_uavs_line(
        self,
        context: SimulationContext,
        uavs: Dict[int, UavState],
        leader_id: int,
        relay_count: int,
    ) -> None:
        if context.ground is None:
            return
        leader = uavs[leader_id]
        if leader.target_goal_id is None:
            return

        goal_position = next(
            goal.position for goal in context.goals if goal.id == leader.target_goal_id
        )
        positions = []
        for index in range(1, relay_count + 2):
            factor = index / (relay_count + 1)
            positions.append(
                Position(
                    x=context.ground.position.x
                    + (goal_position.x - context.ground.position.x) * factor,
                    y=context.ground.position.y
                    + (goal_position.y - context.ground.position.y) * factor,
                )
            )

        free_uavs = [
            uav for uav in uavs.values() if uav.state == "Free" and uav.id != leader_id
        ]
        for position in positions[:-1]:
            if not free_uavs:
                break
            relay = min(
                free_uavs,
                key=lambda uav: distance_between(uav.position, position),
            )
            relay.state = "Relay"
            relay.leader_id = leader.id
            relay.relay_target = position
            leader.slave_ids.add(relay.id)
            free_uavs.remove(relay)

    def get_next_position(
        self,
        current_position: Position,
        target_position: Position,
        speed: float = 1.0,
    ) -> Position:
        dx = target_position.x - current_position.x
        dy = target_position.y - current_position.y
        distance = float(np.hypot(dx, dy))
        if distance == 0 or distance <= speed:
            return target_position
        return Position(
            x=current_position.x + (dx / distance) * speed,
            y=current_position.y + (dy / distance) * speed,
        )


ALGORITHMS = [ConnectivityAwareAlgorithm]
