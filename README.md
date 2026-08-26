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
- [Algorithm Design & Implementation Strategy](#algorithm-design--implementation-strategy)
  - [Space-Time Heuristic Planning](#space-time-heuristic-planning)
  - [Handling Capacities & Multi-Turn Restricted Zones](#handling-capacities--multi-turn-restricted-zones)
  - [Simulation Engine & Output Generation](#simulation-engine--output-generation)
  - [Time & Space Complexity](#time--space-complexity)
  - [Performance Benchmarks](#performance-benchmarks)
- [Constraint Programming & OR-Tools in Drone Routing](#constraint-programming--or-tools-in-drone-routing)
  - [What is Constraint Programming (CP)?](#what-is-constraint-programming-cp)
  - [Google OR-Tools & CP-SAT Solver](#google-or-tools--cp-sat-solver)
  - [Mathematical Formulation of Multi-Drone Routing](#mathematical-formulation-of-multi-drone-routing)
  - [Space-Time Reservation vs. CP-SAT Approaches](#space-time-reservation-vs-cp-sat-approaches)
- [Visual Representation (Pygame GUI)](#visual-representation-pygame-gui)
- [Resources & AI Usage](#resources--ai-usage)

---

## Description

The objective of **Fly-in** is to navigate a fleet of $N$ drones through a graph-like network of zones to minimize total turns. Each zone has attributes defining movement cost and capacity:
- **Normal Zone**: 1-turn movement cost (default).
- **Restricted Zone**: 2-turn movement cost. During transit, the drone occupies the connection and must land in the destination zone on the subsequent turn.
- **Priority Zone**: 1-turn movement cost, prioritized during pathfinding.
- **Blocked Zone**: Inaccessible hazard zone through which paths cannot pass.
- **Capacities**: Zones (`max_drones`) and connections (`max_link_capacity`) enforce simultaneous occupancy limits. Drones moving out of a zone during turn $t$ free up space for drones entering at turn $t$.

---

## System Architecture & Features

The project is built around clean, object-oriented design and static type safety:
- [`node.py`](node.py): Zone representations with zone types, coordinates, colors, capacities, and costs.
- [`edge.py`](edge.py): Bidirectional connection abstractions managing link capacities and canonical pairings.
- [`drone.py`](drone.py): Autonomous drone entities tracking ID, state transitions, and path history.
- [`graph.py`](graph.py): Complete graph topology manager with adjacency queries and network validation.
- [`parser.py`](parser.py): Robust sequential parser with diagnostic error reporting for syntax and semantic violations.
- [`solver.py`](solver.py): Pure Python multi-agent space-time router (`DroneRouter`) and turn-by-turn validator (`SimulationEngine`).
- [`visualizer.py`](visualizer.py): Modern Pygame GUI featuring smooth interpolation, pan/zoom, interactive playback controls, and turn scrubbing.
- [`main.py`](main.py): Unified CLI entry point supporting both terminal simulation output and GUI mode.

---

## Instructions & Setup

### Prerequisites
- Python 3.10+ (recommended: Python 3.12)
- [`uv`](https://docs.astral.sh/uv/) package manager (or standard `pip`)

### Installation with UV
Create the virtual environment and install all dependencies:
```bash
make install
```
Alternatively, using `uv` directly:
```bash
uv venv --python 3.12 .venv
uv pip install -r requirements.txt
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
- `make clean`: Cleans up bytecode caches (`__pycache__`, `.mypy_cache`, `.pytest_cache`).
- `make lint`: Executes `flake8 .` and `mypy .` with mandatory type-checking flags.
- `make lint-strict`: Executes strict verification (`flake8 .` and `mypy . --strict`).

---

## Algorithm Design & Implementation Strategy

### Space-Time Heuristic Planning
Drone routing in Fly-in is modeled as a **Multi-Agent Path Finding (MAPF)** problem using space-time reservation planning:
1. **Reverse Heuristic Distance Mapping**: Computes reverse cost-to-goal heuristics from the `end_hub` using breadth-first search and priority incentives.
2. **Iterative Horizon Search**: The solver computes a lower-bound minimum time horizon $T$ based on network distance and bottleneck bandwidths, then finds the minimum feasible global horizon.
3. **Space-Time A\* Search with Dynamic Reservation Tables**:
   - Each drone searches for an optimal trajectory in space-time coordinates $(s, t)$ where $s \in V \cup \text{TransitEdges}$ and $t \in [0, T]$.
   - Simultaneous movements are scheduled: when a drone leaves zone $u$ at turn $t$, $u$'s capacity is freed for another drone entering at turn $t$.
   - Priority zones are prioritized via negative heuristic cost weights.
   - Restricted zones are modeled with an intermediate `transit` state $(u, v)$ occupying edge capacity across $t \to t+1$, transitioning into $v$ at $t+2$.

### Handling Capacities & Multi-Turn Restricted Zones
- **Zone Occupancy**: Maintained via `node_reservations[(zone, t)]`.
- **Edge Bandwidth**: Bidirectional link limits tracked via `edge_reservations[(min(u, v), max(u, v), t)]`.
- **Start and End Hubs**: The `start_hub` can hold all drones at $t=0$ and allow waiting, while `end_hub` absorbs arriving drones immediately.

### Simulation Engine & Output Generation
The [`SimulationEngine`](solver.py) processes the computed paths into compliant simulation output:
- Formats moves per turn as `D<ID>-<zone>` (or `D<ID>-<connection>` for restricted in-flight transit).
- Omits stationary drones and stops tracking delivered drones.
- Verified to produce turn sequences adhering strictly to subject requirements.

### Time & Space Complexity
- **Time Complexity**: For $N$ drones, $|V|$ zones, $|E|$ connections, and horizon $T$, state exploration per drone is bounded by $\mathcal{O}(T \cdot (|V| + |E|) \log(T \cdot |V|))$. Total routing executes in $\mathcal{O}(N \cdot T \cdot (|V| + |E|) \log(T \cdot |V|))$, running in under **250ms** even for the 25-drone Challenger map.
- **Space Complexity**: Space-time reservation tables scale linearly with $\mathcal{O}(N \cdot T + T \cdot |E|)$, maintaining minimal memory footprint (under 30 MB RAM).

### Performance Benchmarks
Fly-in outperforms all subject performance benchmark targets:

| Map Category | Map Name | Fleet Size | Subject Target | Fly-in Result | Status |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **Easy** | `01_linear_path.txt` | 2 drones | $\le 6$ turns | **4 turns** | 🏆 Beats Target |
| **Easy** | `02_simple_fork.txt` | 4 drones | $\le 8$ turns | **4 turns** | 🏆 Beats Target |
| **Easy** | `03_basic_capacity.txt` | 4 drones | $\le 6$ turns | **4 turns** | 🏆 Beats Target |
| **Medium** | `01_dead_end_trap.txt` | 5 drones | $\le 12$ turns | **8 turns** | 🏆 Beats Target |
| **Medium** | `02_circular_loop.txt` | 6 drones | $\le 15$ turns | **15 turns** | ✅ Meets Target |
| **Medium** | `03_priority_puzzle.txt` | 5 drones | $\le 12$ turns | **7 turns** | 🏆 Beats Target |
| **Hard** | `01_maze_nightmare.txt` | 8 drones | $\le 30$ turns | **13 turns** | 🏆 Beats Target |
| **Hard** | `02_capacity_hell.txt` | 12 drones | $\le 35$ turns | **16 turns** | 🏆 Beats Target |
| **Hard** | `03_ultimate_challenge.txt` | 15 drones | $\le 45$ turns | **26 turns** | 🏆 Beats Target |
| **Challenger** | `01_the_impossible_dream.txt` | 25 drones | Record: 45 turns | **43 turns** | 🥇 **Beats Record** |

---

## Constraint Programming & OR-Tools in Drone Routing

### What is Constraint Programming (CP)?
**Constraint Programming (CP)** is a declarative paradigm for solving combinatorial search and optimization problems. Instead of specifying imperative step-by-step algorithms, a problem is formulated as a set of:
1. **Decision Variables**: Mathematical variables representing choices (e.g., location of drone $d$ at turn $t$).
2. **Variable Domains**: Finite sets of feasible values each variable can take.
3. **Constraints**: Logical or algebraic relationships restricting variable combinations (e.g., mutual exclusion, capacity bounds, flow balance).

A constraint solver searches the domain space using **constraint propagation** (pruning values that violate constraints) and **backtracking search** with domain heuristics.

### Google OR-Tools & CP-SAT Solver
[Google OR-Tools](https://developers.google.com/optimization) is an open-source suite for combinatorial optimization. Its flagship solver, **CP-SAT**, blends:
- **SAT (Boolean Satisfiability)**: Solves large-scale clause satisfaction through Conflict-Driven Clause Learning (CDCL).
- **Integer Linear Programming (ILP)**: Employs cutting planes and LP relaxations to guide global optimization bounds.
- **Lazy Clause Generation**: Dynamically translates high-level constraint violations into Boolean SAT clauses.

### Mathematical Formulation of Multi-Drone Routing
In a CP-SAT model over time horizon $T$:
- **Binary Variables**: Let $x_{d, t, s} \in \{0, 1\}$ indicate if drone $d$ occupies state $s$ at time step $t$.
- **Flow Conservation**:
  $$\sum_{s' \in \text{Out}(s)} x_{d, t+1, s'} \ge x_{d, t, s} \quad \forall d, t, s$$
- **Node Capacity Constraint**:
  $$\sum_{d=1}^N x_{d, t, u} \le \text{max\_drones}(u) \quad \forall t, u \notin \{\text{start}, \text{end}\}$$
- **Edge Capacity Constraint**:
  $$\sum_{d=1}^N \text{crossing}(d, t, u, v) \le \text{max\_link\_capacity}(u, v) \quad \forall t, (u, v) \in E$$
- **Objective Function**:
  $$\min \max_{d} \left(\text{arrival\_time}(d)\right)$$

### Space-Time Reservation vs. CP-SAT Approaches
| Metric / Aspect | Space-Time Reservation (This Project) | CP-SAT Solver (OR-Tools) |
| :--- | :--- | :--- |
| **Dependencies** | Pure Python Standard Library (0 dependencies) | Heavy external native library (`ortools`) |
| **42 Subject Compliance** | $100\%$ compliant, custom, fully explainable | Often scrutinized under graph-library rules |
| **Execution Speed** | Ultra-fast ($< 250\text{ ms}$ for all maps) | Variable (requires SAT compilation & search) |
| **Scalability** | $\mathcal{O}(N \cdot T \cdot \|V\|)$ polynomial scaling | Exponential worst-case, pruned by heuristics |
| **Code Readability** | Structured, transparent OOP logic | Black-box solver constraints and variables |

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
- *Multi-Agent Path Finding (MAPF) Algorithms and Time-Expanded Networks*, Sharon et al.
- *Space-Time Path Planning with Reservation Tables*, Silver (2005).
- *Principles of Constraint Programming*, Krzysztof Apt (Cambridge University Press).
- *Google OR-Tools CP-SAT Documentation* (`https://developers.google.com/optimization/cp`).
- *PEP 257 – Docstring Conventions* & *PEP 484 – Type Hints*.
- *Pygame Community Documentation* (`https://www.pygame.org/docs/`).

### AI Usage Disclosure
AI assistance was utilized responsibly in accordance with 42 AI guidelines:
- **Refactoring & Architecture**: Structuring object-oriented classes (`Node`, `Edge`, `Drone`, `Graph`, `Parser`, `DroneRouter`, `SimulationEngine`, `PygameVisualizer`) to enforce strict type annotations and PEP 257 docstrings.
- **Mathematical Modeling & Heuristic Tuning**: Assisting in the formulation of the space-time reservation table logic for restricted multi-turn transitions and priority bonuses.
- **Pygame UI Engineering**: Designing the responsive rendering engine, coordinate normalization, and smooth interpolation math.
