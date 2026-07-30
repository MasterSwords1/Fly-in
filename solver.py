"""Pathfinding solver using Google OR-Tools CP-SAT.

Formulates drone routing as a constraint programming model to find optimal
paths while respecting node and edge capacities and minimizing total turns.
"""

from typing import Any, Dict, List, Set, Tuple, Optional
from ortools.sat.python import cp_model

from graph import Graph


class OrToolsSolver:
    """Solver class that uses CP-SAT to calculate drone scheduling."""

    def __init__(self, graph: Graph) -> None:
        """Initializes the solver with a graph.

        Args:
            graph: The Graph object representing the network.
        """
        self.graph = graph

    def solve(self) -> List[List[str]]:
        """Solves the routing problem and returns the path for each drone.

        Iteratively increases the time horizon T to find the absolute minimum
        time step where all drones can reach the goal.

        Returns:
            A list of paths, where paths[i] is a list of node names or edge
            names at each time step t for drone i.
        """
        if not self.graph.startNode or not self.graph.endNode:
            return []

        start_node = self.graph.startNode.name
        end_node = self.graph.endNode.name
        num_drones = len(self.graph.drones)

        if num_drones == 0:
            return []

        # Identify physical nodes (ignoring blocked ones)
        physical_nodes: Set[str] = {
            name for name, node in self.graph.nodes.items()
            if node.zone != "blocked"
        }

        # Build virtual nodes representing 2-turn restricted transits
        # virtual_nodes[virtual_name] = (source, destination)
        virtual_nodes: Dict[str, Tuple[str, str]] = {}
        for edge in self.graph.edges:
            u, v = edge.start, edge.end
            if u not in physical_nodes or v not in physical_nodes:
                continue

            node_u = self.graph.nodes[u]
            node_v = self.graph.nodes[v]

            if node_v.zone == "restricted":
                virtual_nodes[f"in_flight_{u}_{v}"] = (u, v)
            if node_u.zone == "restricted":
                virtual_nodes[f"in_flight_{v}_{u}"] = (v, u)

        # Store for use in _to_sim_name
        self._virtual_nodes = virtual_nodes

        # All states in the expanded graph
        states: Set[str] = physical_nodes.union(virtual_nodes.keys())

        # Build incoming and outgoing transition maps
        out_transitions: Dict[str, Set[str]] = {s: set() for s in states}
        in_transitions: Dict[str, Set[str]] = {s: set() for s in states}

        # Transitions for virtual nodes
        for v_node, (src, dest) in virtual_nodes.items():
            out_transitions[src].add(v_node)
            in_transitions[v_node].add(src)

            out_transitions[v_node].add(dest)
            in_transitions[dest].add(v_node)

        # Transitions for physical nodes (staying and 1-turn moves)
        for name in physical_nodes:
            out_transitions[name].add(name)
            in_transitions[name].add(name)

        for edge in self.graph.edges:
            u, v = edge.start, edge.end
            if u not in physical_nodes or v not in physical_nodes:
                continue

            node_u = self.graph.nodes[u]
            node_v = self.graph.nodes[v]

            # 1-turn direct move from u to v (if destination not restricted)
            if node_v.zone != "restricted":
                out_transitions[u].add(v)
                in_transitions[v].add(u)

            # 1-turn direct move from v to u (if destination not restricted)
            if node_u.zone != "restricted":
                out_transitions[v].add(u)
                in_transitions[u].add(v)

        # Find static shortest path distance (turns) from start to end
        dist = 999
        visited = {start_node: 0}
        queue = [start_node]
        while queue:
            curr = queue.pop(0)
            if curr == end_node:
                dist = visited[curr]
                break
            for nxt in out_transitions[curr]:
                if nxt not in visited:
                    visited[nxt] = visited[curr] + 1
                    queue.append(nxt)

        # Sum of outgoing link capacities from start
        c_out = sum(
            edge.capacity for edge in self.graph.edges
            if edge.start == start_node or edge.end == start_node
        )
        if c_out <= 0:
            c_out = 1

        # Dynamic lower bound calculation
        import math
        # If c_out is 1, drones must leave one by one. 
        # First batch (size c_out) takes 'dist' turns to arrive.
        # Total turns = ceil(num_drones / c_out) + dist - 1
        min_t = math.ceil(num_drones / c_out) + dist - 1

        # Check every time step from min_t to find absolute minimum
        for T in range(min_t, min_t + 61): # Try up to 60 additional turns
            # Forward Reachability
            reach_fwd = {0: {start_node}}
            for t in range(1, T + 1):
                prev_set = reach_fwd[t - 1]
                curr_set = set()
                for s1 in prev_set:
                    curr_set.update(out_transitions[s1])
                reach_fwd[t] = curr_set

            # Backward Reachability
            reach_bwd = {T: {end_node}}
            for t in range(T - 1, -1, -1):
                next_set = reach_bwd[t + 1]
                curr_set = set()
                for s2 in next_set:
                    curr_set.update(in_transitions[s2])
                reach_bwd[t] = curr_set

            # Filter active states per time step
            active_states_at = {}
            infeasible = False
            for t in range(T + 1):
                active_states_at[t] = reach_fwd[t].intersection(reach_bwd[t])
                if not active_states_at[t]:
                    infeasible = True
                    break

            if infeasible:
                continue

            # Choose solver strategy
            if num_drones <= 8: # Slightly increased threshold for global
                # Global optimal path routing
                paths = self._solve_global(
                    T, start_node, end_node, num_drones, physical_nodes,
                    virtual_nodes, active_states_at, out_transitions
                )
                if paths:
                    return paths
            else:
                # Fast sequential path routing
                paths = self._solve_sequential(
                    T, start_node, end_node, num_drones, physical_nodes,
                    virtual_nodes, active_states_at, out_transitions
                )
                if paths:
                    return paths

        return []

    def _solve_global(
        self, T: int, start_node: str, end_node: str, num_drones: int,
        physical_nodes: Set[str], virtual_nodes: Dict[str, Tuple[str, str]],
        active_states_at: Dict[int, Set[str]],
        out_transitions: Dict[str, Set[str]]
    ) -> Optional[List[List[str]]]:
        """Runs global solver routing all drones simultaneously."""
        model: Any = cp_model.CpModel()

        # x[d, t, s] = 1 if drone d is in state s at time t
        x: Dict[Tuple[int, int, str], Any] = {}
        for d in range(num_drones):
            for t in range(T + 1):
                for s in active_states_at[t]:
                    x[d, t, s] = model.NewBoolVar(f"x_{d}_{t}_{s}")

        # State constraints
        for d in range(num_drones):
            for t in range(T + 1):
                model.Add(sum(x[d, t, s] for s in active_states_at[t]) == 1)
            model.Add(x[d, 0, start_node] == 1)
            model.Add(x[d, T, end_node] == 1)

        # Transitions
        for d in range(num_drones):
            for t in range(T):
                for s in active_states_at[t]:
                    nxt_states = out_transitions[s].intersection(
                        active_states_at[t + 1]
                    )
                    if nxt_states:
                        model.Add(
                            sum(x[d, t + 1, nxt] for nxt in nxt_states)
                            >= x[d, t, s]
                        )
                    else:
                        model.Add(x[d, t, s] == 0)

        # Physical node capacity limits
        for t in range(T + 1):
            for name in active_states_at[t]:
                if (name == start_node or name == end_node or
                        name not in physical_nodes):
                    continue
                node = self.graph.nodes[name]
                model.Add(
                    sum(x[d, t, name] for d in range(num_drones))
                    <= node.max_drones
                )

        # Connection capacity limits
        for t in range(T):
            for edge in self.graph.edges:
                u, v = edge.start, edge.end
                if u not in physical_nodes or v not in physical_nodes:
                    continue

                crossing_pairs = []
                if self.graph.nodes[v].zone != "restricted":
                    if (u in active_states_at[t] and
                            v in active_states_at[t + 1]):
                        crossing_pairs.append((u, v))
                if self.graph.nodes[u].zone != "restricted":
                    if (v in active_states_at[t] and
                            u in active_states_at[t + 1]):
                        crossing_pairs.append((v, u))
                v_u_v = f"in_flight_{u}_{v}"
                if v_u_v in virtual_nodes:
                    if (u in active_states_at[t] and
                            v_u_v in active_states_at[t + 1]):
                        crossing_pairs.append((u, v_u_v))
                    if (v_u_v in active_states_at[t] and
                            v in active_states_at[t + 1]):
                        crossing_pairs.append((v_u_v, v))
                v_v_u = f"in_flight_{v}_{u}"
                if v_v_u in virtual_nodes:
                    if (v in active_states_at[t] and
                            v_v_u in active_states_at[t + 1]):
                        crossing_pairs.append((v, v_v_u))
                    if (v_v_u in active_states_at[t] and
                            u in active_states_at[t + 1]):
                        crossing_pairs.append((v_v_u, u))

                if not crossing_pairs:
                    continue

                crossing_vars = []
                for d in range(num_drones):
                    for s1, s2 in crossing_pairs:
                        c_var = model.NewBoolVar(
                            f"cross_{d}_{t}_{edge.start}_{edge.end}_{s1}_{s2}"
                        )
                        model.Add(c_var <= x[d, t, s1])
                        model.Add(c_var <= x[d, t + 1, s2])
                        model.Add(
                            x[d, t, s1] + x[d, t + 1, s2] - 1
                            <= c_var
                        )
                        crossing_vars.append(c_var)

                model.Add(sum(crossing_vars) <= edge.capacity)

        # Minimize max arrival time
        arrival_times = []
        for d in range(num_drones):
            arr_t = model.NewIntVar(0, T, f"arr_{d}")
            arrival_times.append(arr_t)
            for t in range(T):
                if end_node in active_states_at[t]:
                    model.Add(arr_t >= t + 1 - T * x[d, t, end_node])
                else:
                    model.Add(arr_t >= t + 1)

        max_turn = model.NewIntVar(0, T, "max_turn")
        model.AddMaxEquality(max_turn, arrival_times)
        
        # Primary objective: minimize max arrival time
        # Secondary objective: minimize sum of arrival times weighted by drone index
        # to break symmetries and prefer D1 moving before D2, etc.
        total_arrival_weight = sum(
            arr_t * (num_drones - d) for d, arr_t in enumerate(arrival_times)
        )
        model.Minimize(max_turn * 1000000 + total_arrival_weight)

        sat_solver: Any = cp_model.CpSolver()
        sat_solver.parameters.max_time_in_seconds = 5.0
        sat_solver.parameters.symmetry_level = 0
        status = sat_solver.Solve(model)

        if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            paths: List[List[str]] = []
            for d in range(num_drones):
                d_path: List[str] = []
                for t in range(T + 1):
                    found_s = ""
                    for s in active_states_at[t]:
                        if sat_solver.Value(x[d, t, s]) == 1:
                            found_s = s
                            break
                    d_path.append(self._to_sim_name(found_s))
                paths.append(d_path)
            return paths

        return None

    def _solve_sequential(
        self, T: int, start_node: str, end_node: str, num_drones: int,
        physical_nodes: Set[str], virtual_nodes: Dict[str, Tuple[str, str]],
        active_states_at: Dict[int, Set[str]],
        out_transitions: Dict[str, Set[str]]
    ) -> Optional[List[List[str]]]:
        """Runs sequential solver routing drones one-by-one."""
        node_reservations: Dict[Tuple[int, str], int] = {}
        edge_reservations: Dict[Tuple[int, Tuple[str, str]], int] = {}

        all_paths: List[List[str]] = []

        for d in range(num_drones):
            model: Any = cp_model.CpModel()
            x: Dict[Tuple[int, str], Any] = {}
            for t in range(T + 1):
                for s in active_states_at[t]:
                    x[t, s] = model.NewBoolVar(f"x_{t}_{s}")

            # State constraints
            for t in range(T + 1):
                model.Add(sum(x[t, s] for s in active_states_at[t]) == 1)
            model.Add(x[0, start_node] == 1)
            model.Add(x[T, end_node] == 1)

            # Transitions
            for t in range(T):
                for s in active_states_at[t]:
                    nxt_states = out_transitions[s].intersection(
                        active_states_at[t + 1]
                    )
                    if nxt_states:
                        model.Add(
                            sum(x[t + 1, nxt] for nxt in nxt_states)
                            >= x[t, s]
                        )
                    else:
                        model.Add(x[t, s] == 0)

            # Node capacities (after removing previous drone reservations)
            for t in range(T + 1):
                for name in active_states_at[t]:
                    if (name == start_node or name == end_node or
                            name not in physical_nodes):
                        continue
                    node = self.graph.nodes[name]
                    reserved = node_reservations.get((t, name), 0)
                    if node.max_drones - reserved <= 0:
                        model.Add(x[t, name] == 0)

            # Connection capacities (after removing reservations)
            for t in range(T):
                for edge in self.graph.edges:
                    u, v = edge.start, edge.end
                    if u not in physical_nodes or v not in physical_nodes:
                        continue

                    reserved = edge_reservations.get(
                        (t, (edge.start, edge.end)), 0
                    )
                    if edge.capacity - reserved <= 0:
                        # Fully blocked, disable crossing transitions
                        if (self.graph.nodes[v].zone != "restricted" and
                                u in active_states_at[t] and
                                v in active_states_at[t + 1]):
                            model.Add(x[t, u] + x[t + 1, v] <= 1)
                        if (self.graph.nodes[u].zone != "restricted" and
                                v in active_states_at[t] and
                                u in active_states_at[t + 1]):
                            model.Add(x[t, v] + x[t + 1, u] <= 1)
                        v_u_v = f"in_flight_{u}_{v}"
                        if v_u_v in virtual_nodes:
                            if (u in active_states_at[t] and
                                    v_u_v in active_states_at[t + 1]):
                                model.Add(x[t, u] + x[t + 1, v_u_v] <= 1)
                            if (v_u_v in active_states_at[t] and
                                    v in active_states_at[t + 1]):
                                model.Add(x[t, v_u_v] + x[t + 1, v] <= 1)
                        v_v_u = f"in_flight_{v}_{u}"
                        if v_v_u in virtual_nodes:
                            if (v in active_states_at[t] and
                                    v_v_u in active_states_at[t + 1]):
                                model.Add(x[t, v] + x[t + 1, v_v_u] <= 1)
                            if (v_v_u in active_states_at[t] and
                                    u in active_states_at[t + 1]):
                                model.Add(x[t, v_v_u] + x[t + 1, u] <= 1)

            # Minimize arrival time
            arr_t = model.NewIntVar(0, T, f"arr_{d}")
            for t in range(T):
                if end_node in active_states_at[t]:
                    model.Add(arr_t >= t + 1 - T * x[t, end_node])
                else:
                    model.Add(arr_t >= t + 1)
            model.Minimize(arr_t)

            sat_solver: Any = cp_model.CpSolver()
            sat_solver.parameters.max_time_in_seconds = 2.0
            status = sat_solver.Solve(model)

            if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                return None

            # Record path
            d_path: List[str] = []
            for t in range(T + 1):
                found_s = ""
                for s in active_states_at[t]:
                    if sat_solver.Value(x[t, s]) == 1:
                        found_s = s
                        break
                d_path.append(found_s)
            all_paths.append([self._to_sim_name(s) for s in d_path])

            # Update node reservations
            for t in range(T + 1):
                s = d_path[t]
                if s in physical_nodes:
                    node_reservations[(t, s)] = (
                        node_reservations.get((t, s), 0) + 1
                    )

            # Update edge reservations
            for t in range(T):
                s1 = d_path[t]
                s2 = d_path[t + 1]
                for edge in self.graph.edges:
                    u, v = edge.start, edge.end
                    crossing = False
                    if (self.graph.nodes[v].zone != "restricted" and
                            s1 == u and s2 == v):
                        crossing = True
                    elif (self.graph.nodes[u].zone != "restricted" and
                          s1 == v and s2 == u):
                        crossing = True
                    elif s1 == u and s2 == f"in_flight_{u}_{v}":
                        crossing = True
                    elif s1 == f"in_flight_{u}_{v}" and s2 == v:
                        crossing = True
                    elif s1 == v and s2 == f"in_flight_{v}_{u}":
                        crossing = True
                    elif s1 == f"in_flight_{v}_{u}" and s2 == u:
                        crossing = True

                    if crossing:
                        edge_reservations[(t, (u, v))] = (
                            edge_reservations.get((t, (u, v)), 0) + 1
                        )

        return all_paths

    def _to_sim_name(self, state: str) -> str:
        """Converts virtual node name back to simulation format."""
        if state in self._virtual_nodes:
            src, dest = self._virtual_nodes[state]
            for edge in self.graph.edges:
                if ((edge.start == src and edge.end == dest) or
                        (edge.start == dest and edge.end == src)):
                    return f"{edge.start}-{edge.end}"
            return f"{src}-{dest}"
        return state


class SimulationEngine:
    """Engine that generates and outputs step-by-step movements from paths."""

    def __init__(self, graph: Graph, paths: List[List[str]]) -> None:
        """Initializes the engine.

        Args:
            graph: The Graph object.
            paths: A list of paths for each drone.
        """
        self.graph = graph
        self.paths = paths
        self.turns: List[List[str]] = []
        self._run_simulation()

    def _run_simulation(self) -> None:
        """Processes paths to construct turn-by-turn movements."""
        if not self.paths or not self.graph.endNode:
            return

        end_name = self.graph.endNode.name
        num_drones = len(self.paths)
        max_len = max(len(p) for p in self.paths)

        # Turn t (1-indexed) corresponds to transition from t-1 to t
        for t in range(1, max_len):
            turn_moves: List[str] = []
            for d_idx in range(num_drones):
                path = self.paths[d_idx]
                if t >= len(path):
                    continue
                prev_pos = path[t - 1]
                curr_pos = path[t]

                # Move if current position differs from previous
                if curr_pos != prev_pos:
                    # Ignore if the drone had already arrived at goal
                    if prev_pos == end_name:
                        continue

                    # Drone IDs in outputs are 1-indexed (e.g. D1)
                    drone_id = (
                        self.graph.drones[d_idx].id
                        if d_idx < len(self.graph.drones) else d_idx
                    )
                    drone_str = f"D{drone_id + 1}"
                    turn_moves.append(f"{drone_str}-{curr_pos}")

            if turn_moves:
                self.turns.append(turn_moves)

    def print_output(self) -> None:
        """Prints the simulation output to stdout."""
        for turn_moves in self.turns:
            print(" ".join(turn_moves))

    def get_total_turns(self) -> int:
        """Returns the total simulation turns."""
        return len(self.turns)
