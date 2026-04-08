from __future__ import annotations

import math

from simulation_api import AlgorithmPlan, GoalAssignment, SimulationAlgorithm, SimulationContext


class UserNearestGoalAlgorithm(SimulationAlgorithm):
    """
    Example custom algorithm.

    Copy this file, rename the class and the `name` field, then replace the
    selection logic inside `build_plan`.
    """

    name = "user_nearest_goal"
    display_name = "User Example: Nearest Goal"
    description = "Ornek kullanici algoritmasi. Bu dosya kopyalanip ozellestirilebilir."

    def build_plan(self, context: SimulationContext) -> AlgorithmPlan:
        free_goals = list(context.free_goals())
        assignments = []

        for uav in context.free_uavs():
            if not free_goals:
                break
            target = min(
                free_goals,
                key=lambda goal: self.distance_between(uav.position, goal.position),
            )
            assignments.append(GoalAssignment(uav_id=uav.id, goal_id=target.id))
            free_goals.remove(target)

        return AlgorithmPlan(goal_assignments=tuple(assignments))

    @staticmethod
    def distance_between(first, second) -> float:
        return math.hypot(first.x - second.x, first.y - second.y)


ALGORITHMS = [UserNearestGoalAlgorithm]
