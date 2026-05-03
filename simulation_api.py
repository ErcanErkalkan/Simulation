"""Stable API for writing custom simulation algorithms.

This module provides a stable, backward-compatible interface for implementing
custom routing algorithms. It exports the key classes and types needed to
write new algorithms for the simulation framework.

The API includes:
- Algorithm context and interfaces
- Data structures for algorithm output (assignments)
- View classes for accessing simulation state
- Position and entity representation

This module is designed to remain stable across versions to ensure
algorithm compatibility.

Example:
    >>> from simulation_api import SimulationAlgorithm, AlgorithmPlan
    >>> 
    >>> class MyAlgorithm(SimulationAlgorithm):
    ...     def get_plan(self, context):
    ...         plan = AlgorithmPlan()
    ...         # Implement algorithm logic
    ...         return plan

Classes:
    SimulationAlgorithm: Base class for custom algorithms
    AlgorithmPlan: Container for algorithm output
    SimulationContext: Read-only view of simulation state
    UavView: UAV entity information
    GoalView: Goal entity information
    GroundView: Ground station information
    Position: 2D position representation
    GoalAssignment: Goal to UAV assignment
    RelayAssignment: Relay UAV assignment
    SlaveAssignment: Slave UAV assignment
"""

from simulation_core import (
    AlgorithmPlan,
    GoalAssignment,
    GoalView,
    GroundView,
    Position,
    RelayAssignment,
    SimulationAlgorithm,
    SimulationContext,
    SlaveAssignment,
    UavView,
)

__all__ = [
    "AlgorithmPlan",
    "GoalAssignment",
    "GoalView",
    "GroundView",
    "Position",
    "RelayAssignment",
    "SimulationAlgorithm",
    "SimulationContext",
    "SlaveAssignment",
    "UavView",
]
