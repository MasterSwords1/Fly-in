*This project has been created as part of the 42 curriculum by ariyad.*

# Fly-in: Autonomous Drone Fleet Pathfinding & Simulation

Fly-in is an optimized, object-oriented multi-agent drone routing and simulation system developed in Python. It routes an arbitrary fleet of autonomous drones across a network of interconnected hubs/zones from a starting base to a designated destination in the minimum possible simulation turns, while strictly adhering to zone occupancy limits, connection bandwidths, and zone-specific movement mechanics.

---

## Table of Contents
- [Description](#description)
- [System Architecture & Features](#system-architecture--features)
- [Instructions & Setup](#instructions--setup)
  - [Prerequisites](#prerequisites)
  - [Installation with UV](#installation-with-uv)
  - [Running the Simulation (CLI Mode)](#running-the-simulation-cli-mode)
  - [Interactive Pygame Graphical Visualizer](#interactive-pygame-graphical-visualizer)
  - [Makefile Commands](#makefile-commands)
- [Constraint Programming & Solver Architecture](#constraint-programming--solver-architecture)
  - [What is Constraint Programming (CP)?](#what-is-constraint-programming-cp)
  - [Google OR-Tools CP-SAT Engine](#google-or-tools-cp-sat-engine)
  - [Mathematical Formulation & Model Design](#mathematical-formulation--model-design)
  - [Virtual In-Flight Transit for Restricted Zones](#virtual-in-flight-transit-for-restricted-zones)
  - [Global vs. Sequential Optimization Strategies](#global-vs-sequential-optimization-strategies)
  - [Performance Benchmarks](#performance-benchmarks)
- [Visual Representation (Pygame GUI)](#visual-representation-pygame-gui)
- [Resources & AI Usage](#resources--ai-usage)

---

## Description

The objective of **Fly-in** is to navigate a fleet of N drones through a network of zones to minimize total turns. Each zone has attributes defining movement cost and capacity:
- **Normal Zone**: 1-turn movement cost (default).
- **Restricted Zone**: 2-turn movement cost. During transit, the drone occupies the connection and must land in the destination zone on the subsequent turn.
- **Priority Zone**: 1-turn movement cost, prioritized during pathfinding.
- **Blocked Zone**: Inaccessible hazard zone through which paths cannot pass.
- **Capacities**: Zones (`max_drones`) and connections (`max_link_capacity`) enforce simultaneous occupancy limits. Drones moving out of a zone during turn `t` free up space for drones entering at turn `t`.

---

## System Architecture & Features

The project is built around clean, object-oriented design and static type safety:
- [`node.py`](node.py): Zone representations with zone types, coordinates, colors, capacities, and costs.
- [`edge.py`](edge.py): Bidirectional connection abstractions managing link capacities and canonical pairings.
- [`drone.py`](drone.py): Autonomous drone entities tracking ID, state transitions, and path history.
- [`graph.py`](graph.py): Complete graph topology manager with adjacency queries and network validation.
- [`parser.py`](parser.py): Robust sequential parser with diagnostic error reporting for syntax and semantic violations.
- [`solver.py`](solver.py): Google OR-Tools CP-SAT constraint programming solver (`OrToolsSolver`) and turn-by-turn validator (`SimulationEngine`).
- [`visualizer.py`](visualizer.py): Modern Pygame GUI featuring smooth interpolation, pan/zoom, interactive playback controls, and turn scrubbing.
- [`main.py`](main.py): Unified CLI entry point supporting both terminal simulation output and GUI mode.

---

## Instructions & Setup

### Prerequisites
- Python 3.10+ (recommended: Python 3.12)
- [`uv`](https://docs.astral.sh/uv/) package manager

### Installation with UV
Create the virtual environment and install all dependencies directly from `pyproject.toml`:
```bash
make install
```
Alternatively, using `uv` directly:
```bash
uv venv --python 3.12 .venv
uv pip install -e .
```

### Running the Simulation (CLI Mode)
To run the simulation in CLI mode and output the turn-by-turn log:
```bash
.venv/bin/python main.py maps/easy/01_linear_path.txt
```
Or use the default make command:
```bash
make run
```

### Interactive Pygame Graphical Visualizer
To launch the interactive graphical user interface using Pygame:
```bash
.venv/bin/python main.py --gui maps/hard/01_maze_nightmare.txt
```
Or directly:
```bash
.venv/bin/python visualizer.py maps/challenger/01_the_impossible_dream.txt
```

### Makefile Commands
The included [`Makefile`](Makefile) automates all project workflows:
- `make install`: Provisions `.venv` and installs dependencies via `uv`.
- `make run`: Executes the simulation on the baseline map.
- `make debug`: Runs the simulation in step-by-step debug mode with Python `pdb`.
- `make clean`: Cleans up bytecode caches (`__pycache__`, `.mypy_cache`, `.pytest_cache`, `*.egg-info`).
- `make lint`: Executes `flake8 .` and `mypy .` with mandatory type-checking flags.
- `make lint-strict`: Executes strict verification (`flake8 .` and `mypy . --strict`).

---

## Constraint Programming & Solver Architecture

### What is Constraint Programming (CP)?
**Constraint Programming (CP)** is a declarative paradigm for solving combinatorial decision and optimization problems. Instead of writing procedural search routines, the problem is modeled as:
1. **Decision Variables**: Mathematical variables that represent decisions (e.g., boolean indicator for whether drone `d` occupies state `s` at turn `t`).
2. **Variable Domains**: Finite sets of permissible values for each variable (e.g., `{0, 1}`).
3. **Constraints**: Mathematical and logical rules restricting simultaneous assignments (e.g., mutual exclusion, flow conservation, capacity limits).

### Google OR-Tools CP-SAT Engine
This project utilizes [Google OR-Tools](https://developers.google.com/optimization), specifically its state-of-the-art **CP-SAT** solver. CP-SAT couples:
- **Conflict-Driven Clause Learning (CDCL)** SAT solving.
- **Integer Linear Programming (ILP)** relaxation and cutting planes.
- **Lazy Clause Generation**, which dynamically compiles domain deductions into boolean clauses during search.

### Mathematical Formulation & Model Design
For a given simulation time horizon `T`:

```text
1. Decision Variables:
   x(d, t, s) in {0, 1}
   Indicates whether drone d occupies state s at turn t (where s in PhysicalNodes U VirtualTransitNodes).

2. State Exclusivity & Boundary Conditions:
   - For all d and t: Sum(x(d, t, s) for all active s) == 1  (each drone is in exactly one state)
   - x(d, 0, start_node) == 1                                (all drones start at start_hub)
   - x(d, T, end_node) == 1                                  (all drones arrive at end_hub by turn T)

3. Flow Transition Constraints:
   For every drone d, turn t in [0, T - 1], and state s:
   Sum(x(d, t + 1, nxt) for all valid reachable states nxt) >= x(d, t, s)

4. Zone Capacity Constraints:
   For every turn t and physical node u (excluding start_hub and end_hub):
   Sum(x(d, t, u) for all drones d) <= max_drones(u)

5. Connection Bandwidth Constraints:
   For every turn t and connection (u, v):
   Sum(crossing_var(d, t, u, v) for all drones d) <= max_link_capacity(u, v)

6. Multi-Objective Function:
   Primary:   Minimize max(arrival_turn of all drones)
   Secondary: Minimize sum of weighted individual arrival turns (breaks symmetries)
```

### Virtual In-Flight Transit for Restricted Zones
Restricted zones take 2 turns to enter. To model this in discrete single-turn transitions without altering time indexing:
- For every edge `(u, v)` where `v` is a restricted zone, a virtual state `in_flight_u_v` is introduced.
- At turn `t`, moving from `u` into transit transitions to `in_flight_u_v` at `t + 1` (consuming connection capacity across `t -> t + 1`).
- From `in_flight_u_v`, the only permissible outgoing transition is into `v` at `t + 2` (consuming connection capacity across `t + 1 -> t + 2`).
- Drones cannot stall or wait on virtual in-flight states, adhering strictly to the subject mandate.

### Global vs. Sequential Optimization Strategies
- **Global Simultaneous Solver (`_solve_global`)**: For fleet sizes `N <= 8`, all drones are routed concurrently in a unified constraint model, guaranteeing global turn optimality.
- **Sequential Reservation Solver (`_solve_sequential`)**: For larger fleets (`N > 8`, up to 25 drones), drones are routed sequentially with dynamic space-time reservations to ensure sub-second solving speed on complex mazes.

### Performance Benchmarks
Fly-in solves all benchmark maps within required targets:

| Map Category | Map Name | Fleet Size | Subject Target | Fly-in Result | Status |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **Easy** | `01_linear_path.txt` | 2 drones | <= 6 turns | **4 turns** | Beats Target |
| **Easy** | `02_simple_fork.txt` | 4 drones | <= 8 turns | **4 turns** | Beats Target |
| **Easy** | `03_basic_capacity.txt` | 4 drones | <= 6 turns | **4 turns** | Beats Target |
| **Medium** | `01_dead_end_trap.txt` | 5 drones | <= 12 turns | **8 turns** | Beats Target |
| **Medium** | `02_circular_loop.txt` | 6 drones | <= 15 turns | **15 turns** | Meets Target |
| **Medium** | `03_priority_puzzle.txt` | 5 drones | <= 12 turns | **7 turns** | Beats Target |
| **Hard** | `01_maze_nightmare.txt` | 8 drones | <= 30 turns | **13 turns** | Beats Target |
| **Hard** | `02_capacity_hell.txt` | 12 drones | <= 35 turns | **16 turns** | Beats Target |
| **Hard** | `03_ultimate_challenge.txt` | 15 drones | <= 45 turns | **26 turns** | Beats Target |
| **Challenger** | `01_the_impossible_dream.txt` | 25 drones | Record: 45 turns | **43 turns** | **Beats Record** |

---

## Visual Representation (Pygame GUI)

The interactive graphical interface provides real-time visual feedback:
- **Zone Display**: Color-coded hubs displaying zone names, capacities, and badges for Restricted (`[R:2t]`), Priority (`[P]`), and Blocked (`[X]`) status.
- **Connection Bandwidth**: Rendered lines with thickness scaling according to `max_link_capacity`.
- **Drone Animation**: Smooth continuous cubic interpolation between turns showing drones flying along connections and parking at hubs.
- **Playback Controls**:
  - `Space`: Play / Pause animation.
  - `Right Arrow` / `Left Arrow`: Step turn forward / backward.
  - `R`: Reset simulation to turn 0.
  - `+` / `-`: Adjust playback speed (0.5x, 1x, 2x, 4x, 8x).
  - `F`: Fit and re-center the map.
  - `Mouse Drag` & `Mouse Wheel`: Interactive Pan & Zoom across large mazes.
  - `Turn Scrubber`: Clickable progress bar to jump directly to any turn.

---

## Resources & AI Usage

### References
- *Principles of Constraint Programming*, Krzysztof Apt (Cambridge University Press).
- *Google OR-Tools CP-SAT Documentation* (`https://developers.google.com/optimization/cp`).
- *Multi-Agent Path Finding (MAPF) Algorithms and Time-Expanded Networks*, Sharon et al.
- *PEP 257 – Docstring Conventions* & *PEP 484 – Type Hints*.
- *Pygame Community Documentation* (`https://www.pygame.org/docs/`).

### AI Usage Disclosure
AI assistance was utilized responsibly in accordance with 42 AI guidelines:
- **Refactoring & Architecture**: Structuring object-oriented classes (`Node`, `Edge`, `Drone`, `Graph`, `Parser`, `OrToolsSolver`, `SimulationEngine`, `PygameVisualizer`) to enforce strict type annotations and PEP 257 docstrings.
- **Constraint Formulation**: Implementing the CP-SAT variable bindings, virtual transit states, and reachability pruning.
- **Pygame UI Engineering**: Designing the responsive rendering engine, coordinate normalization, and smooth interpolation math.
