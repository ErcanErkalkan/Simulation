# User Algorithms

Yeni bir algoritma eklemek icin bu klasore bir `*.py` dosyasi koyun.

Asgari iskelet:

```python
from simulation_api import AlgorithmPlan, GoalAssignment, SimulationAlgorithm, SimulationContext


class MyAlgorithm(SimulationAlgorithm):
    name = "my_algorithm"
    display_name = "My Algorithm"
    description = "Kisa aciklama"

    def build_plan(self, context: SimulationContext) -> AlgorithmPlan:
        free_goals = list(context.free_goals())
        assignments = []
        for uav in context.free_uavs():
            if not free_goals:
                break
            goal = free_goals.pop(0)
            assignments.append(GoalAssignment(uav_id=uav.id, goal_id=goal.id))
        return AlgorithmPlan(goal_assignments=tuple(assignments))


ALGORITHMS = [MyAlgorithm]
```

Notlar:

- `name` benzersiz olmali.
- Dosya adi `_` ile baslamamali.
- Modulu kaydettikten sonra uygulamayi yeniden acinca algoritma listesinde gorunur.
