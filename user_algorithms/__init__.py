"""Custom simulation algorithms developed by users.

This package is where you should place your custom routing algorithms
for the heterogeneous UAV network simulation framework.

To create a custom algorithm:

1. Create a new Python file in this package (e.g., my_algorithm.py)
2. Implement a class that inherits from SimulationAlgorithm
3. Override the get_plan() method with your algorithm logic
4. Register your algorithm with the AlgorithmRegistry

Example Implementation:
    >>> from simulation_api import SimulationAlgorithm, AlgorithmPlan, SimulationContext
    >>> 
    >>> class MyAlgorithm(SimulationAlgorithm):
    ...     @property
    ...     def name(self):
    ...         return "my_algorithm"
    ...     
    ...     def get_plan(self, context: SimulationContext) -> AlgorithmPlan:
    ...         '''Implement your custom algorithm logic here'''
    ...         plan = AlgorithmPlan()
    ...         # Your algorithm implementation...
    ...         return plan
    ...     
    ...     def reset(self):
    ...         '''Reset algorithm state if needed'''
    ...         pass

Key Components:
- SimulationAlgorithm: Base class for all algorithms
- SimulationContext: Read-only access to current simulation state
- AlgorithmPlan: Container for your algorithm's output
- UavView, GoalView, GroundView: Entity state information

See example_user_algorithm.py for a complete working example.
"""
