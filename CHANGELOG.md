# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-05-03

### Added

- Initial public release of the Heterogeneous Network Simulation framework
- Core simulation engine for autonomous flying ad hoc networks (FANETs)
- Modular architecture separating core engine from algorithm implementations
- Algorithm API for implementing custom routing algorithms
- Built-in routing algorithms:
  - Greedy algorithm
  - Current algorithm
- Graphical user interface (GUI) for simulation control and visualization
- Support for real UAV hardware:
  - DJI Tello drone integration via djitellopy
  - CoDrone support via codrone-edu package
- Domain models for:
  - UAVs with position and velocity tracking
  - Goals and goal assignments
  - Ground stations
  - Network topology representation
- Comprehensive unit tests
- Example user algorithm implementation
- Documentation with installation and usage instructions
- Citation files (CITATION.cff and CITATION.bib) for academic references
- Requirements management with optional dependencies for hardware support

### Features

- Simulation of heterogeneous UAV networks with configurable parameters
- Real-time or accelerated simulation execution
- Pluggable algorithm framework for easy algorithm development and testing
- Visualization of UAV positions, goals, and network connections
- Support for multiple drone types in the same simulation
- Hardware abstraction layer for swappable drone implementations
- Configuration management system
- Algorithm registry for dynamic algorithm loading
- Comprehensive logging capabilities

### Dependencies

- Python 3.11+
- numpy >= 1.26
- scipy >= 1.11
- Optional: codrone-edu >= 2.0 (for CoDrone support)
- Optional: djitellopy >= 2.5.0, opencv-python >= 4.8.0 (for DJI Tello support)

### Documentation

- Complete README with installation, usage, and extension instructions
- Inline code documentation
- Example user algorithm
- API documentation in module docstrings
- Related publication: Erkalkan, E., Topuz, V., & Buldu, A. (2024). "Addressing the Return Visit Challenge in Autonomous Flying Ad Hoc Networks Linked to a Central Station." *Sensors*, 24(23), 7859.

### Project Structure

- `simulation_core/`: Core simulation engine and APIs
- `simulation_app/`: Application layer with UI and hardware integration
- `algorithms/`: Built-in routing algorithms
- `user_algorithms/`: User-defined algorithms
- `tests/`: Unit tests
- `CITATION.cff`: Citation metadata in CFF format
- `CITATION.bib`: Citation in BibTeX format
- `pyproject.toml`: Project metadata and dependencies
- `requirements.txt`: Python dependencies

---

## Version History Notes

### Release Criteria for Future Versions

- Comprehensive test coverage (>80%)
- Updated documentation
- Backward compatibility notes in changelog
- Semantic versioning compliance

### Future Directions

Planned enhancements may include:

- GUI improvements and enhanced visualization
- Additional algorithm implementations
- Performance optimization
- Extended hardware support for other drone types
- Distributed simulation capabilities
- Advanced statistical analysis tools
