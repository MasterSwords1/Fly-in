"""Curses-based terminal interface for the Fly-in drone simulation.

The TUI keeps the same core workflow as the graphical interface while using
minimal terminal resources: load a map, solve it, inspect stats, load logs, and
play back turns.
"""

from __future__ import annotations

import curses
import os
import time
from typing import Dict, List, Optional, Tuple

from graph import Graph
from parser import Parser


class SimplePathfinder:
    """A basic shortest-path generator used for the simple simulation mode."""

    def __init__(self, graph: Graph) -> None:
        self.graph = graph

    def find_shortest_path(self) -> Optional[List[str]]:
        if not self.graph.startNode or not self.graph.endNode:
            return None

        start_name = self.graph.startNode.name
        end_name = self.graph.endNode.name

        adjacency: Dict[str, List[str]] = {
            name: [] for name in self.graph.nodes
        }
        for edge in self.graph.edges:
            u, v = edge.start, edge.end
            node_u = self.graph.nodes[u]
            node_v = self.graph.nodes[v]
            if node_u.zone != "blocked" and node_v.zone != "blocked":
                adjacency[u].append(v)
                adjacency[v].append(u)

        queue: List[List[str]] = [[start_name]]
        visited: set[str] = {start_name}

        while queue:
            path = queue.pop(0)
            node = path[-1]
            if node == end_name:
                return path

            for neighbor in adjacency[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(path + [neighbor])

        return None

    def generate_simulation(self) -> List[List[str]]:
        path = self.find_shortest_path()
        if not path or not self.graph.endNode:
            return []

        end_name = self.graph.endNode.name
        turns: List[List[str]] = []
        num_drones = len(self.graph.drones)

        drone_paths: List[List[str]] = []
        for i in range(num_drones):
            d_path: List[str] = [path[0]]
            for _ in range(i):
                d_path.append(path[0])

            for j in range(len(path) - 1):
                u = path[j]
                v = path[j + 1]
                node_v = self.graph.nodes[v]
                edge = None
                for e in self.graph.edges:
                    if ((e.start == u and e.end == v) or
                            (e.start == v and e.end == u)):
                        edge = e
                        break

                if node_v.zone == "restricted" and edge:
                    connection_name = f"{edge.start}-{edge.end}"
                    d_path.append(connection_name)
                    d_path.append(v)
                else:
                    d_path.append(v)

            drone_paths.append(d_path)

        max_len = max(len(p) for p in drone_paths)
        for t in range(1, max_len + 1):
            turn_moves: List[str] = []
            for d_idx in range(num_drones):
                d_path = drone_paths[d_idx]
                if t >= len(d_path):
                    continue
                prev = d_path[t - 1]
                curr = d_path[t]
                if prev != curr and prev != end_name:
                    turn_moves.append(f"D{d_idx + 1}-{curr}")
            if turn_moves:
                turns.append(turn_moves)

        return turns


class TerminalApp:
    """A lightweight terminal UI for loading and replaying simulations."""

    def __init__(
        self, stdscr: curses.window,
        initial_map: Optional[str] = None
    ) -> None:
        self.stdscr = stdscr
        self.graph: Optional[Graph] = None
        self.map_filepath: str = ""
        self.log_turns: List[List[str]] = []
        self.drone_positions_history: List[Dict[str, str]] = []
        self.current_turn = 0
        self.is_playing = False
        self.playback_speed_ms = 500
        self.status_message = (
            "Press o to open a map, m to cycle maps, or q to quit."
        )
        self._available_maps: list[str] = []
        self._map_index = -1
        self._last_tick = time.monotonic()
        self._initial_map = initial_map

    def run(self) -> None:
        curses.curs_set(0)
        self.stdscr.nodelay(True)
        self.stdscr.timeout(50)

        if self._initial_map:
            self.load_map(self._initial_map)

        while True:
            self.render()
            ch = self.stdscr.getch()

            if ch != -1 and self.handle_key(ch):
                break

            if self.is_playing and self.log_turns:
                now = time.monotonic()
                if (now - self._last_tick) * 1000 >= self.playback_speed_ms:
                    self.step_forward()
                    self._last_tick = now

    def handle_key(self, ch: int) -> bool:
        if ch in (ord("q"), 27):
            return True
        if ch in (ord("o"),):
            filepath = self.prompt("Map file path: ")
            if filepath:
                self.load_map(filepath)
        elif ch in (ord("m"),):
            self.load_next_map()
        elif ch in (ord("l"),):
            filepath = self.prompt("Log file path: ")
            if filepath:
                self.load_log(filepath)
        elif ch in (ord("s"),):
            self.run_simple_sim()
        elif ch in (ord("p"), ord(" ")):
            self.toggle_play()
        elif ch in (curses.KEY_RIGHT, ord("n")):
            self.step_forward()
        elif ch in (curses.KEY_LEFT, ord("b")):
            self.step_back()
        elif ch in (curses.KEY_HOME, ord("g")):
            self.go_to_start()
        elif ch in (curses.KEY_END, ord("G")):
            self.go_to_end()
        elif ch in (ord("+"), ord("=")):
            self.change_speed(-100)
        elif ch in (ord("-"), ord("_")):
            self.change_speed(100)
        elif ch in (ord("r"),):
            if self.map_filepath:
                self.load_map(self.map_filepath)
        return False

    def prompt(self, message: str) -> str:
        self.is_playing = False
        self.render()

        max_y, max_x = self.stdscr.getmaxyx()
        input_row = max_y - 1
        prompt_col = min(len(message), max_x - 1)

        curses.echo()
        curses.curs_set(1)
        self.stdscr.nodelay(False)
        try:
            self.stdscr.move(input_row, 0)
            self.stdscr.clrtoeol()
            self.stdscr.addnstr(input_row, 0, message, max_x - 1)
            self.stdscr.refresh()
            data = self.stdscr.getstr(
                input_row, prompt_col, max(1, max_x - prompt_col - 1)
            )
        finally:
            curses.noecho()
            curses.curs_set(0)
            self.stdscr.nodelay(True)

        return data.decode(errors="ignore").strip()

    def ensure_map_list(self) -> None:
        if self._available_maps:
            return

        map_files: list[str] = []
        maps_dir = "./maps"
        if os.path.exists(maps_dir):
            for root_dir, _, files in os.walk(maps_dir):
                for file_name in files:
                    if file_name.endswith(".txt"):
                        map_files.append(
                            os.path.relpath(
                                os.path.join(root_dir, file_name), "."
                            )
                        )

        map_files.sort()
        self._available_maps = map_files

    def load_next_map(self) -> None:
        self.ensure_map_list()
        if not self._available_maps:
            self.status_message = "No map files were found in ./maps."
            return

        if self.map_filepath in self._available_maps:
            self._map_index = self._available_maps.index(self.map_filepath)

        self._map_index = (self._map_index + 1) % len(self._available_maps)
        self.load_map(self._available_maps[self._map_index])

    def load_map(self, filepath: str) -> None:
        try:
            graph = Graph()
            parser = Parser()
            parser.parseFile(graph, filepath)

            self.graph = graph
            self.map_filepath = filepath
            self.clear_log_state()

            self.ensure_map_list()
            if filepath in self._available_maps:
                self._map_index = self._available_maps.index(filepath)

            self._solve_map()
            self.status_message = f"Loaded {os.path.basename(filepath)}"
        except Exception as exc:
            self.status_message = f"Failed to load map: {exc}"

    def _solve_map(self) -> None:
        if not self.graph:
            return

        from solver import OrToolsSolver, SimulationEngine

        solver = OrToolsSolver(self.graph)
        paths = solver.solve()
        if not paths:
            self.status_message = "No valid route found for this map."
            return

        engine = SimulationEngine(self.graph, paths)
        self.set_log_turns(engine.turns)

    def load_log(self, filepath: str) -> None:
        if not self.graph:
            self.status_message = "Load a map before loading a log."
            return

        try:
            turns: List[List[str]] = []
            with open(filepath, "r", encoding="utf-8") as file:
                for line in file:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    turns.append(line.split())

            self.set_log_turns(turns)
            self.status_message = f"Loaded log with {len(turns)} turns."
        except Exception as exc:
            self.status_message = f"Failed to load log: {exc}"

    def run_simple_sim(self) -> None:
        if not self.graph:
            self.status_message = "Load a map first."
            return

        pathfinder = SimplePathfinder(self.graph)
        turns = pathfinder.generate_simulation()
        if not turns:
            self.status_message = "No path found between start and end hub."
            return

        self.set_log_turns(turns)
        self.status_message = (
            f"Generated simple simulation with {len(turns)} turns."
        )

    def clear_log_state(self) -> None:
        self.log_turns = []
        self.drone_positions_history = []
        self.current_turn = 0
        self.is_playing = False

    def set_log_turns(self, turns: List[List[str]]) -> None:
        self.log_turns = turns
        self.current_turn = 0
        self.is_playing = False
        self.drone_positions_history = []

        start_node_name = "start"
        num_drones = 0
        if self.graph:
            if self.graph.startNode:
                start_node_name = self.graph.startNode.name
            num_drones = len(self.graph.drones)

        positions: Dict[str, str] = {}
        for i in range(1, num_drones + 1):
            positions[f"D{i}"] = start_node_name

        self.drone_positions_history.append(positions.copy())

        for turn_moves in self.log_turns:
            for move in turn_moves:
                if "-" not in move:
                    continue
                drone_name, dest = move.split("-", 1)
                if drone_name not in positions:
                    positions[drone_name] = start_node_name
                positions[drone_name] = dest
            self.drone_positions_history.append(positions.copy())

    def get_current_positions(self) -> Dict[str, str]:
        if self.current_turn < len(self.drone_positions_history):
            return self.drone_positions_history[self.current_turn]

        start_name = ""
        if self.graph and self.graph.startNode:
            start_name = self.graph.startNode.name
        num_drones = len(self.graph.drones) if self.graph else 0
        return {f"D{i}": start_name for i in range(1, num_drones + 1)}

    def step_forward(self) -> None:
        if self.current_turn < len(self.log_turns):
            self.current_turn += 1

    def step_back(self) -> None:
        if self.current_turn > 0:
            self.current_turn -= 1

    def go_to_start(self) -> None:
        self.current_turn = 0

    def go_to_end(self) -> None:
        self.current_turn = len(self.log_turns)

    def toggle_play(self) -> None:
        self.is_playing = not self.is_playing
        self._last_tick = time.monotonic()

    def change_speed(self, delta_ms: int) -> None:
        self.playback_speed_ms = max(
            100, min(2000, self.playback_speed_ms + delta_ms)
        )

    def render(self) -> None:
        self.stdscr.erase()
        max_y, max_x = self.stdscr.getmaxyx()

        header_lines = self.build_header_lines(max_x)
        map_lines = self.build_map_lines(max_x, max_y)

        row = 0
        for line in header_lines:
            if row >= max_y - 1:
                break
            self.stdscr.addnstr(row, 0, line, max_x - 1)
            row += 1

        for line in map_lines:
            if row >= max_y - 1:
                break
            self.stdscr.addnstr(row, 0, line, max_x - 1)
            row += 1

        if row < max_y - 1:
            self.stdscr.addnstr(row, 0, "-" * max(0, max_x - 1), max_x - 1)
            row += 1

        footer = self.build_footer_line(max_x)
        self.stdscr.addnstr(max_y - 1, 0, footer, max_x - 1)
        self.stdscr.refresh()

    def build_header_lines(self, width: int) -> List[str]:
        lines: List[str] = []
        lines.append("Fly-in TUI")
        lines.append(
            "Controls: o open map | m next map | l load log | s simple sim | "
            "p play/pause | arrows step | g home | G end | +/- speed | q quit"
        )
        lines.append(self.status_message)
        lines.append("")

        if not self.graph:
            lines.append("No map loaded yet.")
            return [line[: max(0, width - 1)] for line in lines]

        map_name = os.path.basename(self.map_filepath)
        num_nodes = len(self.graph.nodes)
        num_edges = len(self.graph.edges)
        num_drones = len(self.graph.drones)
        lines.append(
            f"Map: {map_name} | Nodes: {num_nodes} | "
            f"Edges: {num_edges} | Drones: {num_drones}"
        )

        if self.log_turns:
            current_positions = self.get_current_positions()
            end_node_name = (
                self.graph.endNode.name if self.graph.endNode else ""
            )
            at_dest = sum(
                1 for pos in current_positions.values() if pos == end_node_name
            )
            in_transit = num_drones - at_dest
            lines.append(
                f"Turn: {self.current_turn}/{len(self.log_turns)} | "
                f"At destination: {at_dest} | In flight: {in_transit} | "
                f"Speed: {self.playback_speed_ms} ms"
            )
            if self.current_turn < len(self.log_turns):
                current_moves = " ".join(self.log_turns[self.current_turn])
                lines.append("Current moves: " + current_moves)
            else:
                lines.append("Current moves: <complete>")

            node_lines = ["Node occupancy:"]
            for name, node in self.graph.nodes.items():
                if node.is_start or node.is_end:
                    continue
                drones_here = sum(
                    1 for pos in current_positions.values() if pos == name
                )
                if drones_here:
                    node_lines.append(
                        f"  {name}: {drones_here}/{node.max_drones} "
                        f"({node.zone})"
                    )
            if len(node_lines) == 1:
                node_lines.append("  none")
            lines.extend(node_lines)
        else:
            lines.append(f"Turn: 0/0 | Speed: {self.playback_speed_ms} ms")
            lines.append("Current moves: <none>")

        return [line[: max(0, width - 1)] for line in lines]

    def build_map_lines(self, width: int, height: int) -> List[str]:
        if not self.graph or not self.graph.nodes:
            return []

        available_height = max(4, height - 12)
        available_width = max(20, width - 2)
        grid_height = min(available_height, 16)
        grid_width = min(available_width, 80)

        grid = [[" " for _ in range(grid_width)] for _ in range(grid_height)]
        positions = self.compute_grid_positions(grid_width, grid_height)

        for edge in self.graph.edges:
            start = positions.get(edge.start)
            end = positions.get(edge.end)
            if not start or not end:
                continue
            for x, y in self.bresenham_line(
                start[0], start[1], end[0], end[1]
            ):
                if (
                    0 <= y < grid_height and 0 <= x < grid_width and
                    grid[y][x] == " "
                ):
                    grid[y][x] = "."

        current_positions = self.get_current_positions()
        node_occupancy: Dict[str, List[str]] = {
            name: [] for name in self.graph.nodes
        }
        for drone, pos in current_positions.items():
            if pos in node_occupancy:
                node_occupancy[pos].append(drone)

        for name, node in self.graph.nodes.items():
            x, y = positions[name]
            label = name[:3]
            if node.is_start:
                label = label.upper()
            elif node.is_end:
                label = label.lower()
            elif node.zone == "restricted":
                label = label[:2] + "R"
            elif node.zone == "priority":
                label = label[:2] + "P"
            elif node.zone == "blocked":
                label = label[:2] + "B"

            self.write_label(grid, x, y, label)
            if node_occupancy.get(name):
                count = str(len(node_occupancy[name]))
                self.write_label(grid, x, min(grid_height - 1, y + 1), count)

        lines = ["Map view:"]
        for row in grid:
            lines.append("".join(row).rstrip())
        return lines

    def compute_grid_positions(
        self, width: int, height: int
    ) -> Dict[str, Tuple[int, int]]:
        x_coords = [node.x for node in self.graph.nodes.values()]
        y_coords = [node.y for node in self.graph.nodes.values()]

        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        range_x = max(max_x - min_x, 1)
        range_y = max(max_y - min_y, 1)

        positions: Dict[str, Tuple[int, int]] = {}
        for name, node in self.graph.nodes.items():
            x = int((node.x - min_x) / range_x * max(width - 1, 1))
            y = int((node.y - min_y) / range_y * max(height - 1, 1))
            y = (height - 1) - y
            positions[name] = (
                max(0, min(width - 1, x)),
                max(0, min(height - 1, y)),
            )
        return positions

    def bresenham_line(
        self, x1: int, y1: int, x2: int, y2: int
    ) -> List[Tuple[int, int]]:
        points: List[Tuple[int, int]] = []
        dx = abs(x2 - x1)
        dy = -abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx + dy
        x, y = x1, y1

        while True:
            points.append((x, y))
            if x == x2 and y == y2:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

        return points

    def write_label(
        self, grid: List[List[str]], x: int, y: int, label: str
    ) -> None:
        if not grid:
            return
        width = len(grid[0])
        if y < 0 or y >= len(grid):
            return
        start_x = max(0, min(width - 1, x - len(label) // 2))
        for offset, char in enumerate(label):
            pos_x = start_x + offset
            if 0 <= pos_x < width:
                grid[y][pos_x] = char

    def build_footer_line(self, width: int) -> str:
        mode = "Playing" if self.is_playing else "Paused"
        progress = f"{self.current_turn}/{len(self.log_turns)}"
        footer = (
            f"{mode} | Turn {progress} | Speed {self.playback_speed_ms} ms"
        )
        return footer[: max(0, width - 1)]


def main(initial_map: Optional[str] = None) -> None:
    curses.wrapper(lambda stdscr: TerminalApp(stdscr, initial_map).run())


if __name__ == "__main__":
    import sys

    map_file = sys.argv[1] if len(sys.argv) > 1 else None
    main(map_file)
