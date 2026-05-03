"""Simulation core module for heterogeneous UAV networks.

This package contains the core simulation engine and related components for
simulating autonomous flying ad hoc networks (FANETs).

Key Components:
- SimulationEngine: Main engine that orchestrates simulation execution
- SimulationEnvironment: Manages simulation state (UAVs, goals, ground station)
- SimulationAlgorithm: Base class for implementing routing algorithms
- SimulationConfig: Configuration management for simulations
- AlgorithmRegistry: Registry for managing algorithm implementations

The API is organized around:
- Core simulation entities (Engine, Environment, Context)
- Algorithm framework (SimulationAlgorithm, AlgorithmPlan)
- Data structures (UavView, GoalView, GroundView, Position)
- Assignments (GoalAssignment, RelayAssignment, SlaveAssignment)

Example:
    >>> from simulation_core import SimulationEngine, AlgorithmRegistry
    >>> from algorithms import GreedyAlgorithm
    >>> 
    >>> registry = AlgorithmRegistry()
    >>> registry.register("greedy", GreedyAlgorithm)
    >>> 
    >>> engine = SimulationEngine()
    >>> engine.set_algorithm("greedy")
    >>> engine.step()
"""

from simulation_core.algorithm_api import (
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
from simulation_core.config import SimulationConfig
from simulation_core.engine import SimulationEngine
from simulation_core.environment import SimulationEnvironment
from simulation_core.registry import AlgorithmRegistry

__all__ = [
    "AlgorithmPlan",
    "AlgorithmRegistry",
    "GoalAssignment",
    "GoalView",
    "GroundView",
    "Position",
    "RelayAssignment",
    "SimulationAlgorithm",
    "SimulationConfig",
    "SimulationContext",
    "SimulationEngine",
    "SimulationEnvironment",
    "SlaveAssignment",
    "UavView",
]
