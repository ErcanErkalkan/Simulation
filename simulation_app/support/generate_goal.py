import random
from typing import List

from simulation_app.domain.goal import Goal
from simulation_app.domain.vector import Vector


class GenerateGoal:
    MARGIN = 30  # Default margin
    MIN_DISTANCE = 50  # Minimum distance between goals

    @staticmethod
    def run(count: int, canvas_width: int, canvas_height: int, margin: int = MARGIN) -> List[Goal]:
        if count <= 0:
            raise ValueError("The count must be a positive integer.")

        if canvas_width <= margin * 2 or canvas_height <= margin * 2:
            raise ValueError(f"Canvas dimensions must be at least {margin * 2} pixels in both width and height.")

        GenerateGoal.MARGIN = margin
        goals: List[Goal] = []
        for i in range(count):
            while True:
                x = random.uniform(margin, canvas_width - margin)
                y = random.uniform(margin, canvas_height - margin)
                pos = Vector(x, y)

                # Ensure no overlap with existing goals
                if all(pos.distance_to(goal.pos) > GenerateGoal.MIN_DISTANCE for goal in goals):
                    break

            goal = Goal(pos=pos, goal_no=i + 1)
            goals.append(goal)

        return goals
