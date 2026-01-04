# Simulation Environment for Return-Visit-Aware FANET Routing (Central Station)

This repository contains the **Python/Tkinter simulation environment** used in the paper:

> **E. Erkalkan, V. Topuz, A. Buldu**, “Addressing the Return Visit Challenge in Autonomous Flying Ad Hoc Networks Linked to a Central Station,” *Sensors*, 24(23), 7859, 2024. DOI: **10.3390/s24237859**

The simulator focuses on **multi-UAV (FANET) connectivity-aware tasking** with a **central station (ground)** and supports both:
- **Single-visit** target assignment (classic “visit once” setting)
- **Revisit-aware** target selection (return-visit challenge) using a **composite valuation** function

## Key capabilities
- **Connectivity-aware coordination** (monitoring network fragmentation)
- **Relay allocation / intervention** when fragmentation is detected
- **Leader / slave roles** and leader handover logic
- **Revisit-aware valuation** for target prioritization (threshold-based)
- **GUI-based scenario creation** (add/generate UAVs, goals, ground station)
- **Scenario save/load** as JSON
- **Simulation logging** (network disconnects, relay interventions, leader changes)
- Optional hooks for **real drone integration** (CoDrone EDU / DJI Tello) *(hardware required)*

## Quick start

### 1) Create a virtual environment
```bash
python -m venv .venv
# Windows
.\\.venv\\Scripts\\activate
# macOS/Linux
source .venv/bin/activate
```

### 2) Install dependencies
This repo contains a `requirements.txt`, but it may reflect a broad development environment.
For **software-only simulation**, you typically only need:
```bash
pip install numpy scipy
```

Optional (only if you plan to use real drone bindings):
```bash
pip install codrone-edu djitellopy
```

### 3) Run the GUI simulator
From the repository root (this folder):
```bash
python ground_main.py
```

## Using the GUI
1. **Generate Ground**: set X/Y and create the central station.
2. **Generate UAVs / Goals**: choose counts and generate random positions.
3. Choose **Target evaluation mode**:
   - **single_visit**: assigns free UAVs to free goals using distance-based assignment.
   - **revisit**: prioritizes revisitable goals based on time-since-last-visit and thresholds.
4. Press **Start** to begin. Use **Save/Load** to persist or restore scenarios.

### Scenario files (JSON)
The GUI can save and load scenarios with this structure:
```json
{
  "Ground": {"X": 100, "Y": 100},
  "UAVs": [{"X": 120, "Y": 140}],
  "Goals": [{"X": 500, "Y": 320}]
}
```

## Outputs and logs
- `simulation_log.txt` records notable events.
- Example historical logs are included (e.g., `A1.txt`, `B1.txt`, …) demonstrating relay interventions and fragmentation handling.

## Project structure
- `ground_main.py` — GUI entrypoint (Tkinter)
- `simulation_engine.py` — core simulation logic
- `valuation.py` — composite valuation for revisit-aware selection
- `functions.py` — adjacency, connectivity checks (connected components), utilities
- `uav.py`, `goal.py`, `ground.py` — entity models
- `SimulationLogger.py` — event logging

## Citation
If you use this code in academic work, please cite the paper and (optionally) this repository.
A ready-to-use citation is provided in `CITATION.cff`.

**BibTeX (paper):**
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

## License
This project is released under the **MIT License** (see `LICENSE`).

---

### Notes
This is research software shared for reproducibility and extension. If you spot issues or want to contribute improvements (e.g., batch-run scripts, cleaner dependency pinning), feel free to open an issue or PR.
