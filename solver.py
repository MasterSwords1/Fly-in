"""Solver module providing optimal multi-agent drone routing and simulation."""

from collections import defaultdict, deque
import heapq
from typing import Dict, Final, List, Optional, Tuple
from graph import Graph
from node import Node

INFINITY: Final[int] = 10**9


class RoutingError(Exception):
    """Exception raised when pathfinding fails or network is unsolvable."""


class DroneRouter:
    """Calculates conflict-free, turn-minimized paths for a fleet of drones.

    Uses space-time heuristic search on time-expanded reservation tables with
    capacity constraints, priority zone incentives, and multi-turn restricted transit.
    """

    def __init__(self, graph: Graph) -> None:
        """Initializes the router with a target Graph.

        Args:
            graph: The Graph object containing network topology and drones.
        """
        self.graph = graph

    def solve(self) -> List[List[str]]:
        """Solves the multi-agent pathfinding problem for all drones in the graph.

        Iteratively searches for the minimal simulation horizon T that allows
        all drones to reach the destination hub while respecting zone and link capacities.

        Returns:
            A list of paths, where path[i] is the sequence of state/zone names at each turn.

        Raises:
            RoutingError: If start/end hubs are missing or destination is unreachable.
        """
        if not self.graph.start_node or not self.graph.end_node:
            raise RoutingError("Start hub or end hub is not defined in graph")

        num_drones = len(self.graph.drones)
        if num_drones == 0:
            return []

        start_name = self.graph.start_node.name
        end_name = self.graph.end_node.name

        # Exclude blocked zones
        valid_nodes: Dict[str, Node] = {
            name: node for name, node in self.graph.nodes.items()
            if not node.is_blocked()
        }

        if start_name not in valid_nodes:
            raise RoutingError(f"Start hub '{start_name}' is blocked")
        if end_name not in valid_nodes:
            raise RoutingError(f"End hub '{end_name}' is blocked")

        # Build adjacency and capacity tables
        adj: Dict[str, List[str]] = defaultdict(list)
        edge_caps: Dict[Tuple[str, str], int] = {}
        for edge in self.graph.edges:
            if edge.start in valid_nodes and edge.end in valid_nodes:
                adj[edge.start].append(edge.end)
                adj[edge.end].append(edge.start)
                edge_caps[(edge.start, edge.end)] = edge.capacity
                edge_caps[(edge.end, edge.start)] = edge.capacity

        # Heuristic reverse distance table from end_hub
        dist_to_goal: Dict[str, int] = {}
        queue: deque[str] = deque([end_name])
        dist_to_goal[end_name] = 0

        while queue:
            curr = queue.popleft()
            for neighbor in adj[curr]:
                step_cost = valid_nodes[curr].cost
                if neighbor not in dist_to_goal:
                    dist_to_goal[neighbor] = dist_to_goal[curr] + step_cost
                    queue.append(neighbor)

        if start_name not in dist_to_goal:
            raise RoutingError(f"Target hub '{end_name}' is unreachable from start '{start_name}'")

        min_dist = dist_to_goal[start_name]
        start_cap = sum(edge_caps.get((start_name, nbr), 1) for nbr in adj[start_name])
        lower_bound_t = min_dist + (num_drones + max(1, start_cap) - 1) // max(1, start_cap) - 1

        # Search over increasing time horizon T
        max_search_horizon = lower_bound_t + 120
        for horizon in range(min_dist, max_search_horizon):
            paths = self._plan_space_time_paths(
                horizon=horizon,
                start_name=start_name,
                end_name=end_name,
                num_drones=num_drones,
                valid_nodes=valid_nodes,
                adj=adj,
                edge_caps=edge_caps,
                dist_to_goal=dist_to_goal,
            )
            if paths is not None and len(paths) == num_drones:
                return paths

        raise RoutingError(f"Could not find a valid routing within {max_search_horizon} turns")

    def _plan_space_time_paths(
        self,
        horizon: int,
        start_name: str,
        end_name: str,
        num_drones: int,
        valid_nodes: Dict[str, Node],
        adj: Dict[str, List[str]],
        edge_caps: Dict[Tuple[str, str], int],
        dist_to_goal: Dict[str, int],
    ) -> Optional[List[List[str]]]:
        """Plans collision-free trajectories for all drones for a fixed horizon.

        Args:
            horizon: Maximum allowed time step.
            start_name: Name of starting hub.
            end_name: Name of destination hub.
            num_drones: Total number of drones to route.
            valid_nodes: Dictionary of accessible nodes.
            adj: Adjacency list mapping node name to neighbor names.
            edge_caps: Dictionary mapping directed edge pairs to capacities.
            dist_to_goal: Precomputed heuristic distance to goal.

        Returns:
            List of path sequences if feasible, None otherwise.
        """
        node_reservations: Dict[Tuple[str, int], int] = defaultdict(int)
        edge_reservations: Dict[Tuple[str, str, int], int] = defaultdict(int)
        all_drone_paths: List[List[str]] = []

        for drone_idx in range(num_drones):
            h0 = dist_to_goal.get(start_name, 0)
            # Min-heap elements: (f_score, turn, current_state, path_history)
            open_set: List[Tuple[int, int, str | Tuple[str, str, str], List[str]]] = []
            heapq.heappush(open_set, (h0 * 100, 0, start_name, [start_name]))
            visited: Dict[Tuple[str | Tuple[str, str, str], int], int] = {}

            found_path: Optional[List[str]] = None

            while open_set:
                f_score, t, curr_state, history = heapq.heappop(open_set)

                if curr_state == end_name:
                    found_path = history
                    break

                if t >= horizon:
                    continue

                state_key = (curr_state, t)
                if state_key in visited and visited[state_key] <= f_score:
                    continue
                visited[state_key] = f_score

                # Case 1: Drone is currently in-transit toward a restricted zone
                if isinstance(curr_state, tuple) and curr_state[0] == "transit":
                    _, u_from, v_dest = curr_state
                    v_node = valid_nodes[v_dest]
                    can_enter_v = (
                        v_dest == end_name or
                        node_reservations[(v_dest, t + 1)] < v_node.max_drones
                    )
                    edge_key = (min(u_from, v_dest), max(u_from, v_dest), t)
                    can_use_edge = edge_reservations[edge_key] < edge_caps.get((u_from, v_dest), 1)

                    if can_enter_v and can_use_edge:
                        h_val = dist_to_goal.get(v_dest, 0)
                        penalty = 80 if v_node.is_priority() else 100
                        heapq.heappush(
                            open_set,
                            ((t + 1 + h_val) * 100 + penalty, t + 1, v_dest, history + [v_dest])
                        )
                    continue

                # Case 2: Drone is at a physical zone
                assert isinstance(curr_state, str)
                u = curr_state

                # Prioritize neighbors: priority zones first, then lowest distance heuristic
                neighbors = sorted(
                    adj[u],
                    key=lambda nbr: (
                        0 if valid_nodes[nbr].is_priority() else 1,
                        dist_to_goal.get(nbr, INFINITY)
                    )
                )

                for v in neighbors:
                    v_node = valid_nodes[v]
                    edge_key = (min(u, v), max(u, v), t)
                    if edge_reservations[edge_key] >= edge_caps.get((u, v), 1):
                        continue

                    # Connection name format
                    conn_name = f"{u}-{v}"
                    edge_obj = self.graph.get_edge(u, v)
                    if edge_obj is not None:
                        conn_name = edge_obj.name

                    if v_node.is_restricted():
                        transit_state = ("transit", u, v)
                        h_val = dist_to_goal.get(v, 0) + 1
                        heapq.heappush(
                            open_set,
                            ((t + 1 + h_val) * 100 + 150, t + 1, transit_state, history + [conn_name])
                        )
                    else:
                        if v == end_name or node_reservations[(v, t + 1)] < v_node.max_drones:
                            h_val = dist_to_goal.get(v, 0)
                            penalty = 80 if v_node.is_priority() else 100
                            heapq.heappush(
                                open_set,
                                ((t + 1 + h_val) * 100 + penalty, t + 1, v, history + [v])
                            )

                # Option to stay / wait at current zone
                u_node = valid_nodes[u]
                can_stay = (
                    u == start_name or
                    u == end_name or
                    node_reservations[(u, t + 1)] < u_node.max_drones
                )
                if can_stay:
                    h_val = dist_to_goal.get(u, 0)
                    wait_penalty = 105 if u == start_name else 115
                    heapq.heappush(
                        open_set,
                        ((t + 1 + h_val) * 100 + wait_penalty, t + 1, u, history + [u])
                    )

            if found_path is None:
                return None

            # Register reservations along the found trajectory
            for step_t in range(len(found_path)):
                st = found_path[step_t]
                if st in valid_nodes and st != start_name and st != end_name:
                    node_reservations[(st, step_t)] += 1

                if step_t < len(found_path) - 1:
                    nxt = found_path[step_t + 1]
                    if st in valid_nodes and nxt in valid_nodes:
                        if st != nxt:
                            edge_k = (min(st, nxt), max(st, nxt), step_t)
                            edge_reservations[edge_k] += 1
                    elif st in valid_nodes and "-" in nxt:
                        u_p, v_p = nxt.split("-", 1)
                        edge_k = (min(u_p, v_p), max(u_p, v_p), step_t)
                        edge_reservations[edge_k] += 1
                    elif "-" in st and nxt in valid_nodes:
                        u_p, v_p = st.split("-", 1)
                        edge_k = (min(u_p, v_p), max(u_p, v_p), step_t)
                        edge_reservations[edge_k] += 1

            all_drone_paths.append(found_path)

        return all_drone_paths


# Backward compatibility alias
OrToolsSolver = DroneRouter


class SimulationEngine:
    """Simulates turn-by-turn drone execution and produces compliant output logs."""

    def __init__(self, graph: Graph, paths: List[List[str]]) -> None:
        """Initializes the simulation engine with graph topology and paths.

        Args:
            graph: The Graph object.
            paths: The list of paths for each drone.
        """
        self.graph = graph
        self.paths = paths
        self.turns: List[List[str]] = []
        self._simulate()

    def _simulate(self) -> None:
        """Constructs the discrete turn-by-turn movement events."""
        if not self.paths or not self.graph.end_node:
            return

        end_name = self.graph.end_node.name
        num_drones = len(self.paths)
        max_duration = max(len(p) for p in self.paths)

        for t in range(1, max_duration):
            turn_moves: List[str] = []
            for d_idx in range(num_drones):
                path = self.paths[d_idx]
                if t >= len(path):
                    continue

                prev_state = path[t - 1]
                curr_state = path[t]

                # If drone moved to a different state
                if curr_state != prev_state:
                    # Omit if drone had already arrived at goal in previous turn
                    if prev_state == end_name:
                        continue

                    drone_id = self.graph.drones[d_idx].id if d_idx < len(self.graph.drones) else d_idx
                    drone_label = f"D{drone_id + 1}"
                    turn_moves.append(f"{drone_label}-{curr_state}")

            if turn_moves:
                self.turns.append(turn_moves)

    def get_turns(self) -> List[List[str]]:
        """Returns the list of space-separated move records per turn.

        Returns:
            List of lists of movement strings.
        """
        return self.turns

    def get_total_turns(self) -> int:
        """Returns the total number of simulation turns.

        Returns:
            Total turn count integer.
        """
        return len(self.turns)

    def print_output(self) -> None:
        """Prints the compliant step-by-step movements to standard output."""
        for turn_actions in self.turns:
            print(" ".join(turn_actions))
