from __future__ import annotations

from algorithms.base import AlgorithmPlan, GoalAssignment, SimulationAlgorithm
from simulation_app.support.valuation import Valuation
from simulation_core.algorithm_api import GoalView, SimulationContext, UavView
from simulation_core.topology import distance_between


class GreedyDistanceAlgorithm(SimulationAlgorithm):
    name = "greedy_distance"
    display_name = "Greedy Distance"
    description = "En yakin serbest hedefe gider, relay mantigi kullanmaz."

    def build_plan(self, context: SimulationContext) -> AlgorithmPlan:
        free_goals = list(context.free_goals())
        assignments = []

        for uav in context.uavs:
            if uav.state != "Free" or not free_goals:
                continue
            target = self.select_goal(uav, free_goals, context)
            assignments.append(GoalAssignment(uav_id=uav.id, goal_id=target.id))
            free_goals.remove(target)

        return AlgorithmPlan(goal_assignments=tuple(assignments))

    def select_goal(
        self,
        uav: UavView,
        goals: list[GoalView],
        context: SimulationContext,
    ) -> GoalView:
        if context.config.target_eval_mode == "revisit":
            return max(
                goals,
                key=lambda goal: Valuation.composite_valuation(
                    context.config.threshold2 + 1
                    if goal.last_visited_time is None
                    else max(context.current_time - goal.last_visited_time, 0.0),
                    uav.position.to_vector(),
                    goal.position.to_vector(),
                    context.config.threshold1,
                    context.config.threshold2,
                ),
            )
        return min(goals, key=lambda goal: distance_between(uav.position, goal.position))


ALGORITHMS = [GreedyDistanceAlgorithm]
