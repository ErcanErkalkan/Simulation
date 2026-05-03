"""Built-in routing algorithms for UAV simulations.

This package contains reference implementations of routing algorithms for
heterogeneous UAV networks. Algorithms are discovered dynamically by the
AlgorithmRegistry system.

Included Algorithms:
- GreedyAlgorithm: Greedy assignment algorithm for task allocation
- CurrentAlgorithm: Current/baseline algorithm implementation

Base Classes:
- SimulationAlgorithm: Abstract base class for all algorithms

Algorithms are registered and loaded at runtime through the AlgorithmRegistry.
To implement a custom algorithm, subclass SimulationAlgorithm and register
it with the registry.

Example:
    >>> from algorithms import GreedyAlgorithm
    >>> from simulation_core import AlgorithmRegistry
    >>> 
    >>> registry = AlgorithmRegistry()
    >>> registry.register("my_algo", GreedyAlgorithm)
"""

# Algorithms are discovered dynamically by AlgorithmRegistry.
