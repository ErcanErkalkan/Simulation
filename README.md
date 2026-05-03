# Heterogeneous Network Simulation for Return-Visit-Aware FANET Routing

A modular simulation environment for autonomous flying ad hoc networks (FANETs) with pluggable routing algorithms and support for both simulated and real-world UAV hardware.

## 🎯 Overview

This project provides a flexible simulation framework for studying heterogeneous UAV networks. It separates the core simulation engine from algorithm implementations, allowing easy development and testing of new routing strategies. The architecture supports both simulated environments and real drone hardware (DJI Tello, CoDrone).

## ✨ Features

- **Modular Architecture**: Clean separation between core simulation engine, domain models, and algorithms
- **Pluggable Algorithms**: Easy to implement and test new routing algorithms
- **Multi-UAV Support**: Simulate heterogeneous networks with different drone types
- **Real Hardware Integration**: Support for DJI Tello and CoDrone drones
- **Graphical User Interface**: Interactive simulation visualization and control
- **Comprehensive Testing**: Unit tests for core functionality
- **Publication-Ready**: Cite in your research using provided CITATION files

## 📁 Project Structure

```
├── simulation_core/        # Core simulation engine and APIs
│   ├── engine.py          # Main simulation engine
│   ├── environment.py     # Simulation environment management
│   ├── algorithm_api.py   # Algorithm interface definitions
│   ├── config.py          # Configuration management
│   ├── registry.py        # Algorithm registry
│   └── topology.py        # Network topology
├── simulation_app/         # Application layer
│   ├── domain/            # Domain entities (UAV, Ground, Goal, Vector)
│   ├── ui/                # Graphical user interface
│   ├── hardware/          # Hardware integrations (Tello, CoDrone)
│   └── support/           # Utility functions and helpers
├── algorithms/             # Built-in routing algorithms
│   ├── base.py            # Base algorithm class
│   ├── current.py         # Current algorithm
│   └── greedy.py          # Greedy algorithm
├── user_algorithms/        # User-defined algorithms
├── tests/                  # Unit tests
└── docs/                   # Documentation (if available)
```

## 🚀 Installation

### Prerequisites

- Python 3.11 or higher
- pip or conda package manager

### Basic Installation

```bash
# Clone the repository
git clone https://github.com/ErcanErkalkan/Simulation.git
cd Simulation

# Install in development mode
pip install -e .
```

### With Optional Hardware Support

```bash
# With CoDrone support
pip install -e ".[codrone]"

# With DJI Tello support
pip install -e ".[tello]"

# With all optional dependencies
pip install -e ".[codrone,tello]"
```

## 📖 Usage

### Running the Graphical User Interface

```bash
# Method 1: Direct execution
python ground_main.py

# Method 2: Using installed command (after pip install -e .)
simulation-ui
```

### Running Simulations Programmatically

```python
from simulation_core import SimulationEngine, SimulationConfig, AlgorithmRegistry
from algorithms import GreedyAlgorithm

# Create configuration
config = SimulationConfig(
    area_width=1000,
    area_height=1000,
    num_uavs=5,
    num_goals=10,
    uav_speed=2.0
)

# Initialize engine and register algorithm
registry = AlgorithmRegistry()
registry.register("greedy", GreedyAlgorithm)

engine = SimulationEngine(config, algorithm="greedy")

# Run simulation
for step in range(100):
    engine.step()
    print(f"Step {step}: {engine.get_statistics()}")
```

### Running Tests

```bash
# Run all tests
python -m unittest -v

# Run specific test module
python -m unittest tests.test_simulation -v

# Run with pytest (if installed)
pytest tests/ -v
```

## 🎮 UI Controls

The graphical interface includes:

- **Simulation Parameters**: Configure UAV count, goals, simulation area size
- **Speed Control**: Controls how many units UAVs move per simulation step
- **Algorithm Selection**: Choose from available routing algorithms
- **Hardware Mode**: Toggle between simulation and real drone mode
- **Visualization**: Real-time view of UAV positions and network topology

### Important Notes

- The `Speed` field determines how many units UAVs advance per simulation step
- Real CoDrone hardware requires the `codrone-edu` package
- Real DJI Tello hardware requires the `djitellopy` package

## 🧪 Testing

The project includes comprehensive unit tests covering:

- Core simulation engine functionality
- Algorithm implementations
- Domain entity operations
- Configuration management

Run tests with:

```bash
python -m unittest discover tests -v
```

## 📚 Extending the Framework

### Creating a Custom Algorithm

1. Create a new file in `user_algorithms/`:

```python
from simulation_core import SimulationAlgorithm, AlgorithmPlan

class MyCustomAlgorithm(SimulationAlgorithm):
    """Custom routing algorithm."""
    
    def get_plan(self, context):
        """Generate allocation plan for current simulation state."""
        # Implement your algorithm logic here
        plan = AlgorithmPlan()
        # ... populate plan ...
        return plan
```

2. Register and use it in the simulation

3. See `user_algorithms/example_user_algorithm.py` for a complete example

## 📰 Citation

If you use this simulation framework in your research, please cite:

```bibtex
@article{erkalkan2024returnvisit,
  title   = {Addressing the Return Visit Challenge in Autonomous Flying Ad Hoc Networks Linked to a Central Station},
  author  = {Erkalkan, Ercan and Topuz, Vedat and Buldu, Ali},
  journal = {Sensors},
  volume  = {24},
  number  = {23},
  pages   = {7859},
  year    = {2024},
  doi     = {10.3390/s24237859}
}
```

## 📋 Requirements

### Core Dependencies
- numpy >= 1.26
- scipy >= 1.11

### Optional Dependencies
- codrone-edu >= 2.0 (for CoDrone hardware support)
- djitellopy >= 2.5.0 and opencv-python >= 4.8.0 (for DJI Tello support)

See `requirements.txt` for full dependency list.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

Contributions are welcome! To contribute:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 👨‍💼 Author

**Ercan Erkalkan**

- GitHub: [@ErcanErkalkan](https://github.com/ErcanErkalkan)
- Research: Autonomous Flying Networks, UAV Routing

## 📚 Related Publications

- Erkalkan, E., Topuz, V., & Buldu, A. (2024). "Addressing the Return Visit Challenge in Autonomous Flying Ad Hoc Networks Linked to a Central Station." *Sensors*, 24(23), 7859.

## 🙏 Acknowledgments

This research was conducted as part of PhD studies in Computer Engineering at [University Name].

## 📞 Support

For questions, issues, or suggestions:
- Open an issue on GitHub
- Contact the author directly

---

**Last Updated**: 2026-05-03
**Version**: 0.1.0
