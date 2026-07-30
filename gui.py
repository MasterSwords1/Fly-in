"""Tkinter-based graphical interface for the Fly-in drone simulation.

Provides interactive canvas visualization, Graphviz rendering, and simulation
playback controls (play/pause, step forward/backward, speed, timeline).
"""

import math
import os
import subprocess
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Any, Dict, List, Set, Tuple, Optional
from PIL import Image, ImageTk

from graph import Graph
from parser import Parser
from solver import OrToolsSolver, SimulationEngine


class SimplePathfinder:
    """A basic pathfinder to generate simple simulations for testing.

    Routes drones along the shortest path, staggering departure times to
    help respect capacities where possible.
    """

    def __init__(self, graph: Graph) -> None:
        """Initializes the pathfinder with a graph.

        Args:
            graph: The Graph object to pathfind on.
        """
        self.graph = graph

    def find_shortest_path(self) -> Optional[List[str]]:
        """Finds the shortest path from start_hub to end_hub ignoring capacity.

        Returns:
            A list of node names representing the path, or None if no path.
        """
        if not self.graph.startNode or not self.graph.endNode:
            return None

        start_name = self.graph.startNode.name
        end_name = self.graph.endNode.name

        # Build adjacency
        adj: Dict[str, List[str]] = {name: [] for name in self.graph.nodes}
        for edge in self.graph.edges:
            u, v = edge.start, edge.end
            node_u = self.graph.nodes[u]
            node_v = self.graph.nodes[v]
            if node_u.zone != "blocked" and node_v.zone != "blocked":
                adj[u].append(v)
                adj[v].append(u)

        # BFS
        queue: List[List[str]] = [[start_name]]
        visited: Set[str] = {start_name}

        while queue:
            path = queue.pop(0)
            node = path[-1]
            if node == end_name:
                return path

            for neighbor in adj[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(path + [neighbor])

        return None

    def generate_simulation(self) -> List[List[str]]:
        """Generates a simple turn-by-turn simulation log.

        Returns:
            A list of turns, where each turn is a list of movement strings
            in the format 'D<id>-<dest>'.
        """
        path = self.find_shortest_path()
        if not path or not self.graph.endNode:
            return []

        end_name = self.graph.endNode.name
        turns: List[List[str]] = []
        num_drones = len(self.graph.drones)

        # Pre-calculate paths for each drone
        # Drone i starts at t = i (staggered)
        # We trace its positions at each time step
        drone_paths: List[List[str]] = []
        for i in range(num_drones):
            d_path: List[str] = [path[0]]
            # Wait at start node until departure
            for _ in range(i):
                d_path.append(path[0])

            # Move along the path
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
                    # Restricted takes 2 turns
                    connection_name = f"{edge.start}-{edge.end}"
                    d_path.append(connection_name)
                    d_path.append(v)
                else:
                    # Normal / Priority takes 1 turn
                    d_path.append(v)

            drone_paths.append(d_path)

        # Convert drone paths to turn-by-turn movements
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


class ZoomPanCanvas(tk.Frame):
    """A Tkinter Frame containing a Canvas with zoom and pan capabilities."""

    def __init__(self, parent: tk.Widget, **kwargs: Any) -> None:
        """Initializes the zoomable and pannable canvas.

        Args:
            parent: The parent widget.
            **kwargs: Extra arguments for the frame.
        """
        super().__init__(parent, **kwargs)
        self.canvas = tk.Canvas(self, bg="#11111b", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.canvas.bind("<ButtonPress-1>", self.start_pan)
        self.canvas.bind("<B1-Motion>", self.pan)
        self.canvas.bind("<Button-4>", self.zoom_in)
        self.canvas.bind("<Button-5>", self.zoom_out)
        self.canvas.bind("<MouseWheel>", self.zoom_wheel)

        self.scale = 1.0
        self.pan_start_x = 0
        self.pan_start_y = 0

    def start_pan(self, event: tk.Event) -> None:
        """Starts the panning operation on click.

        Args:
            event: The Tkinter event object.
        """
        self.canvas.scan_mark(event.x, event.y)
        self.pan_start_x = event.x
        self.pan_start_y = event.y

    def pan(self, event: tk.Event) -> None:
        """Pans the canvas content on drag.

        Args:
            event: The Tkinter event object.
        """
        self.canvas.scan_dragto(event.x, event.y, gain=1)

    def zoom_in(self, event: tk.Event) -> None:
        """Zooms in on mouse scroll up (Linux).

        Args:
            event: The Tkinter event object.
        """
        self.zoom(1.1, event.x, event.y)

    def zoom_out(self, event: tk.Event) -> None:
        """Zooms out on mouse scroll down (Linux).

        Args:
            event: The Tkinter event object.
        """
        self.zoom(0.9, event.x, event.y)

    def zoom_wheel(self, event: tk.Event) -> None:
        """Zooms in/out on mouse wheel scroll (Windows/macOS).

        Args:
            event: The Tkinter event object.
        """
        if event.delta > 0:
            self.zoom(1.1, event.x, event.y)
        else:
            self.zoom(0.9, event.x, event.y)

    def zoom(self, factor: float, x: int, y: int) -> None:
        """Scales all items on the canvas.

        Args:
            factor: The zoom factor.
            x: The anchor X coordinate.
            y: The anchor Y coordinate.
        """
        self.scale *= factor
        self.canvas.scale("all", x, y, factor, factor)


class DroneVisualizerApp:
    """The main Tkinter application class for drone visualization."""

    def __init__(
        self, root: tk.Tk, initial_map: Optional[str] = None
    ) -> None:
        """Initializes the application, styles the UI, and builds the layout.

        Args:
            root: The root Tk window.
            initial_map: Optional initial map file path to load.
        """
        self.root = root
        self.root.title("Fly-in Drone Simulation Visualizer")
        self.root.geometry("1200x800")
        self.root.configure(bg="#1e1e2e")

        self.graph: Optional[Graph] = None
        self.map_filepath: str = ""
        self.log_turns: List[List[str]] = []
        self.drone_positions_history: List[Dict[str, str]] = []

        self.current_turn = 0
        self.is_playing = False
        self.playback_speed = 500  # ms per step
        self.timer_id: Optional[str] = None

        # Node coordinates cache for rendering
        self.node_positions: Dict[str, Tuple[float, float]] = {}

        self._apply_styles()
        self._build_layout()
        self._scan_maps()

        if initial_map:
            self.load_map(initial_map)

    def _apply_styles(self) -> None:
        """Applies custom dark styles to ttk widgets."""
        style = ttk.Style()
        style.theme_use("clam")

        # Main background colors
        style.configure(".", background="#1e1e2e", foreground="#cdd6f4")
        style.configure("TFrame", background="#1e1e2e")
        style.configure(
            "TLabel", background="#1e1e2e", foreground="#cdd6f4",
            font=("Inter", 10)
        )

        # Tabs styling
        style.configure("TNotebook", background="#1e1e2e", borderwidth=0)
        style.configure(
            "TNotebook.Tab", background="#252538", foreground="#cdd6f4",
            borderwidth=0, padding=(10, 5)
        )
        style.map(
            "TNotebook.Tab", background=[("selected", "#11111b")],
            foreground=[("selected", "#89b4fa")]
        )

        # Buttons styling
        style.configure(
            "TButton", background="#252538", foreground="#cdd6f4",
            borderwidth=1, relief="flat", font=("Inter", 9, "bold")
        )
        style.map(
            "TButton", background=[("active", "#313244"),
                                   ("pressed", "#11111b")],
            foreground=[("active", "#89b4fa")]
        )

        # Combobox styling
        style.configure(
            "TCombobox", fieldbackground="#252538",
            background="#252538", foreground="#cdd6f4"
        )

        # Highlight label
        style.configure(
            "Title.TLabel", font=("Inter", 14, "bold"),
            foreground="#89b4fa"
        )
        style.configure(
            "Stat.TLabel", font=("Inter", 10, "bold"),
            foreground="#fab387"
        )

    def _build_layout(self) -> None:
        """Constructs the sidebar, visualizer notebooks, and
        control elements."""
        # Top level paned window to separate controls and visualizer
        self.paned = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        self.paned.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left Sidebar (Controls)
        self.sidebar = ttk.Frame(
            self.paned, width=260, padding=10, relief="solid"
        )
        self.paned.add(self.sidebar, weight=0)

        # Title
        title_lbl = ttk.Label(
            self.sidebar, text="FLY-IN SIMULATOR", style="Title.TLabel"
        )
        title_lbl.pack(fill=tk.X, pady=(0, 15))

        # Map Selector Frame
        map_frame = ttk.LabelFrame(
            self.sidebar, text=" Map Selection ", padding=8
        )
        map_frame.pack(fill=tk.X, pady=(0, 10))

        self.map_combo = ttk.Combobox(map_frame, state="readonly")
        self.map_combo.pack(fill=tk.X, pady=(0, 5))
        self.map_combo.bind("<<ComboboxSelected>>", self.on_map_selected)

        self.btn_load_map = ttk.Button(
            map_frame, text="Browse Map File...", command=self.browse_map
        )
        self.btn_load_map.pack(fill=tk.X)

        # Simulation Log Frame
        log_frame = ttk.LabelFrame(
            self.sidebar, text=" Simulation Log ", padding=8
        )
        log_frame.pack(fill=tk.X, pady=(0, 10))

        self.btn_load_log = ttk.Button(
            log_frame, text="Load Log File...", command=self.browse_log
        )
        self.btn_load_log.pack(fill=tk.X, pady=(0, 5))

        self.btn_simple_sim = ttk.Button(
            log_frame, text="Generate Simple Simulation",
            command=self.run_simple_sim
        )
        self.btn_simple_sim.pack(fill=tk.X, pady=(0, 5))

        self.btn_clear_log = ttk.Button(
            log_frame, text="Clear Log", command=self.clear_log
        )
        self.btn_clear_log.pack(fill=tk.X)

        # Playback Controls Frame
        playback_frame = ttk.LabelFrame(
            self.sidebar, text=" Playback Controls ", padding=8
        )
        playback_frame.pack(fill=tk.X, pady=(0, 10))

        # Buttons grid
        btn_grid = ttk.Frame(playback_frame)
        btn_grid.pack(fill=tk.X, pady=(0, 5))

        self.btn_prev = ttk.Button(
            btn_grid, text="⏮", width=4, command=self.go_to_start
        )
        self.btn_prev.grid(row=0, column=0, padx=2)

        self.btn_step_back = ttk.Button(
            btn_grid, text="◀", width=4, command=self.step_back
        )
        self.btn_step_back.grid(row=0, column=1, padx=2)

        self.btn_play = ttk.Button(
            btn_grid, text="▶ Play", width=10, command=self.toggle_play
        )
        self.btn_play.grid(row=0, column=2, padx=2)

        self.btn_step_fwd = ttk.Button(
            btn_grid, text="▶", width=4, command=self.step_forward
        )
        self.btn_step_fwd.grid(row=0, column=3, padx=2)

        self.btn_next = ttk.Button(
            btn_grid, text="⏭", width=4, command=self.go_to_end
        )
        self.btn_next.grid(row=0, column=4, padx=2)

        # Speed control
        speed_lbl = ttk.Label(playback_frame, text="Speed (ms/step):")
        speed_lbl.pack(anchor=tk.W, pady=(5, 0))
        self.speed_scale = ttk.Scale(
            playback_frame, from_=100, to=2000, value=500,
            command=self.change_speed
        )
        self.speed_scale.pack(fill=tk.X, pady=(0, 5))

        # Timeline Slider
        time_lbl = ttk.Label(playback_frame, text="Timeline:")
        time_lbl.pack(anchor=tk.W, pady=(5, 0))
        self.time_slider = ttk.Scale(
            playback_frame, from_=0, to=0, value=0,
            command=self.on_timeline_slide
        )
        self.time_slider.pack(fill=tk.X)

        # Statistics / Info Panel
        stats_frame = ttk.LabelFrame(
            self.sidebar, text=" Statistics ", padding=8
        )
        stats_frame.pack(fill=tk.BOTH, expand=True)

        self.stats_text = tk.Text(
            stats_frame, bg="#11111b", fg="#cdd6f4",
            font=("Courier", 9), wrap=tk.WORD, borderwidth=0,
            highlightthickness=0
        )
        self.stats_text.pack(fill=tk.BOTH, expand=True)
        self.stats_text.insert(tk.END, "No map loaded.\n")
        self.stats_text.configure(state=tk.DISABLED)

        # Right Workspace (Tabs)
        self.notebook = ttk.Notebook(self.paned)
        self.paned.add(self.notebook, weight=1)

        # Tab 1: Interactive Canvas
        self.canvas_container = ZoomPanCanvas(self.notebook)
        self.notebook.add(self.canvas_container, text=" Interactive Canvas ")

        # Tab 2: Graphviz View
        self.graphviz_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.graphviz_frame, text=" Graphviz View ")

        # Scrollable image canvas for Graphviz
        self.gv_canvas = tk.Canvas(self.graphviz_frame, bg="#11111b")
        self.gv_canvas.pack(fill=tk.BOTH, expand=True)

        self.gv_scroll_x = ttk.Scrollbar(
            self.graphviz_frame, orient=tk.HORIZONTAL,
            command=self.gv_canvas.xview
        )
        self.gv_scroll_x.pack(fill=tk.X, side=tk.BOTTOM)
        self.gv_scroll_y = ttk.Scrollbar(
            self.graphviz_frame, orient=tk.VERTICAL,
            command=self.gv_canvas.yview
        )
        self.gv_scroll_y.pack(fill=tk.Y, side=tk.RIGHT)

        self.gv_canvas.configure(
            xscrollcommand=self.gv_scroll_x.set,
            yscrollcommand=self.gv_scroll_y.set
        )
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)

    def _scan_maps(self) -> None:
        """Scans './maps' recursively for TXT files to populate selector."""
        map_files: List[str] = []
        maps_dir = "./maps"
        if os.path.exists(maps_dir):
            for root_dir, _, files in os.walk(maps_dir):
                for file in files:
                    if file.endswith(".txt"):
                        rel_path = os.path.relpath(
                            os.path.join(root_dir, file), "."
                        )
                        map_files.append(rel_path)

        map_files.sort()
        self.map_combo["values"] = map_files
        if map_files:
            self.map_combo.current(0)
            self.load_map(map_files[0])

    def browse_map(self) -> None:
        """Opens a file dialog to browse for a custom map file."""
        filepath = filedialog.askopenfilename(
            initialdir="./maps",
            title="Select Map File",
            filetypes=(("Text Files", "*.txt"), ("All Files", "*.*"))
        )
        if filepath:
            self.load_map(filepath)

    def browse_log(self) -> None:
        """Opens a file dialog to browse for a simulation log file."""
        filepath = filedialog.askopenfilename(
            title="Select Simulation Log File",
            filetypes=(("Text Files", "*.txt"), ("All Files", "*.*"))
        )
        if filepath:
            self.load_log(filepath)

    def load_map(self, filepath: str) -> None:
        """Parses the map file, auto-solves, and resets visualization.

        Args:
            filepath: Path to the map file.
        """
        try:
            self.map_filepath = filepath
            self.graph = Graph()
            parser = Parser()
            parser.parseFile(self.graph, filepath)

            # Update combobox display if not selected from it
            rel_path = os.path.relpath(filepath, ".")
            if rel_path in self.map_combo["values"]:
                idx = self.map_combo["values"].index(rel_path)
                self.map_combo.current(idx)

            self.clear_log()
            self.calculate_node_rendering_coordinates()

            # Auto-solve: run OR-Tools solver and load results
            self._auto_solve()

            self.draw_interactive_graph()
            self.update_stats()
            self.update_graphviz_view()

        except Exception as e:
            messagebox.showerror(
                "Error Loading Map",
                f"Failed to parse map file:\n{e}"
            )

    def _auto_solve(self) -> None:
        """Runs the OR-Tools solver and loads the solution.

        Prints map_name and turn-by-turn logs to stdout as required
        by the subject output format.
        """
        if not self.graph:
            return

        solver = OrToolsSolver(self.graph)
        paths = solver.solve()
        if not paths:
            return

        engine = SimulationEngine(self.graph, paths)

        # Print map_name and logs to stdout
        map_name = os.path.basename(self.map_filepath)
        print(map_name)
        engine.print_output()

        # Load the solution turns into the GUI simulation
        self.set_log_turns(engine.turns)

    def on_map_selected(self, event: tk.Event) -> None:
        """Triggered when a map is selected from the combobox.

        Args:
            event: The Tkinter event object.
        """
        selected = self.map_combo.get()
        if selected:
            self.load_map(selected)

    def calculate_node_rendering_coordinates(self) -> None:
        """Scales the integer graph coordinates to fit nicely on the canvas."""
        if not self.graph or not self.graph.nodes:
            return

        x_coords = [node.x for node in self.graph.nodes.values()]
        y_coords = [node.y for node in self.graph.nodes.values()]

        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)

        # Scale parameters
        width = 700
        height = 600
        padding = 60

        range_x = max_x - min_x if max_x != min_x else 1
        range_y = max_y - min_y if max_y != min_y else 1

        self.node_positions.clear()
        for name, node in self.graph.nodes.items():
            # Invert Y to match traditional math coordinates where up
            # is positive
            cx = padding + (node.x - min_x) / range_x * (width - 2 * padding)
            cy = height - (
                padding + (node.y - min_y) / range_y * (height - 2 * padding)
            )
            self.node_positions[name] = (cx, cy)

    def load_log(self, filepath: str) -> None:
        """Loads and parses a simulation log file.

        Args:
            filepath: Path to the log file.
        """
        if not self.graph:
            messagebox.showwarning("Warning", "Please load a map first!")
            return

        try:
            turns: List[List[str]] = []
            with open(filepath, "r") as file:
                for line in file:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    turns.append(line.split())

            self.set_log_turns(turns)
            messagebox.showinfo(
                "Log Loaded",
                f"Successfully loaded simulation with {len(turns)} turns."
            )

        except Exception as e:
            messagebox.showerror(
                "Error Loading Log",
                f"Failed to parse simulation log:\n{e}"
            )

    def run_simple_sim(self) -> None:
        """Generates and loads a simple BFS staggered simulation."""
        if not self.graph:
            messagebox.showwarning("Warning", "Please load a map first!")
            return

        pathfinder = SimplePathfinder(self.graph)
        turns = pathfinder.generate_simulation()
        if not turns:
            messagebox.showerror(
                "Error", "No path found between start and end hub!"
            )
            return

        self.set_log_turns(turns)
        messagebox.showinfo(
            "Simple Simulation",
            f"Staggered shortest-path simulation generated: "
            f"{len(turns)} turns."
        )

    def set_log_turns(self, turns: List[List[str]]) -> None:
        """Sets the turns list and pre-computes all drone states.

        Args:
            turns: A list of turns, each containing move strings.
        """
        self.log_turns = turns
        self.current_turn = 0
        self.is_playing = False
        self.btn_play.configure(text="▶ Play")

        # Initialize timeline slider
        total_turns = len(self.log_turns)
        self.time_slider.configure(to=total_turns)
        self.time_slider.set(0)

        # Precompute drone positions
        self.drone_positions_history.clear()

        # Start state at t = 0: All drones at start Node
        start_node_name = "start"
        num_drones = 0
        if self.graph:
            if self.graph.startNode:
                start_node_name = self.graph.startNode.name
            num_drones = len(self.graph.drones)

        positions: Dict[str, str] = {}
        # Prepopulate with standard D1, D2 etc.
        for i in range(1, num_drones + 1):
            positions[f"D{i}"] = start_node_name

        # Reconstruct positions turn by turn
        self.drone_positions_history.append(positions.copy())

        for turn_idx, turn_moves in enumerate(self.log_turns):
            for move in turn_moves:
                if "-" not in move:
                    continue
                drone_name, dest = move.split("-", 1)
                # If we encounter an uninitialized drone, start it at start_hub
                if drone_name not in positions:
                    positions[drone_name] = start_node_name
                positions[drone_name] = dest
            self.drone_positions_history.append(positions.copy())

        self.draw_interactive_graph()
        self.update_stats()

    def clear_log(self) -> None:
        """Clears the loaded simulation log and resets states."""
        self.log_turns.clear()
        self.drone_positions_history.clear()
        self.current_turn = 0
        self.is_playing = False
        self.btn_play.configure(text="▶ Play")
        self.time_slider.configure(to=0)
        self.time_slider.set(0)
        if self.timer_id:
            self.root.after_cancel(self.timer_id)
            self.timer_id = None

        if self.graph:
            self.draw_interactive_graph()
            self.update_stats()

    def get_current_positions(self) -> Dict[str, str]:
        """Returns the mapping of drone names to their current positions.

        Returns:
            A dictionary mapping drone name to node name or edge string.
        """
        if self.current_turn < len(self.drone_positions_history):
            return self.drone_positions_history[self.current_turn]

        start_name = ""
        if self.graph and self.graph.startNode:
            start_name = self.graph.startNode.name
        num_drones = len(self.graph.drones) if self.graph else 0
        return {f"D{i}": start_name for i in range(1, num_drones + 1)}

    def draw_interactive_graph(self) -> None:
        """Renders graph connections, nodes, and active drones on Canvas."""
        if not self.graph:
            return

        canvas = self.canvas_container.canvas
        canvas.delete("all")

        # Keep track of active / occupied nodes and edges
        current_positions = self.get_current_positions()

        # Calculate occupancies
        node_occupancy: Dict[str, List[str]] = {
            name: [] for name in self.graph.nodes
        }
        edge_occupancy: Dict[Tuple[str, str], List[str]] = {}

        for drone, pos in current_positions.items():
            if pos in node_occupancy:
                node_occupancy[pos].append(drone)
            elif "-" in pos:
                # In-flight on edge
                nodes = pos.split("-")
                if len(nodes) == 2:
                    key = (min(nodes[0], nodes[1]), max(nodes[0], nodes[1]))
                    if key not in edge_occupancy:
                        edge_occupancy[key] = []
                    edge_occupancy[key].append(drone)

        # Draw Edges (Connections)
        for edge in self.graph.edges:
            u, v = edge.start, edge.end
            if u not in self.node_positions or v not in self.node_positions:
                continue

            x1, y1 = self.node_positions[u]
            x2, y2 = self.node_positions[v]

            # Highlighting connection if traversed
            edge_key = (min(u, v), max(u, v))
            drones_on_edge = edge_occupancy.get(edge_key, [])

            color = "#313244"
            width = 2

            if drones_on_edge:
                color = "#fab387"  # orange for transit
                width = 4

            # Draw the connection line
            canvas.create_line(
                x1, y1, x2, y2, fill=color, width=width, tags="edge"
            )

            # Draw edge capacity text at midpoint
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            if edge.capacity > 1:
                # Offset slightly from the line
                dx, dy = x2 - x1, y2 - y1
                length = math.hypot(dx, dy) or 1
                nx, ny = -dy / length * 12, dx / length * 12
                canvas.create_text(
                    mx + nx, my + ny, text=f"c:{edge.capacity}",
                    fill="#a6adc8", font=("Inter", 8)
                )

        # Draw Nodes (Hubs)
        node_radius = 24
        for name, node in self.graph.nodes.items():
            if name not in self.node_positions:
                continue

            cx, cy = self.node_positions[name]

            # Base colors for zone types
            fill_color = "#1e1e2e"
            outline_color = "#89b4fa"

            if node.is_start:
                fill_color = "#2e3f2f"
                outline_color = "#a6e3a1"  # green
            elif node.is_end:
                fill_color = "#4c323c"
                outline_color = "#f38ba8"  # red
            elif node.zone == "restricted":
                fill_color = "#453128"
                outline_color = "#fab387"  # orange
            elif node.zone == "priority":
                fill_color = "#392c47"
                outline_color = "#cba6f7"  # purple
            elif node.zone == "blocked":
                fill_color = "#313244"
                outline_color = "#585b70"  # gray

            # Override with custom color if specified in metadata
            if node.color:
                color_map = {
                    "green": "#a6e3a1",
                    "red": "#f38ba8",
                    "blue": "#89b4fa",
                    "yellow": "#f9e2af",
                    "orange": "#fab387",
                    "purple": "#cba6f7",
                    "gray": "#585b70",
                    "black": "#11111b",
                    "brown": "#b48972",
                    "maroon": "#eba0ac",
                    "gold": "#f9e2af",
                    "violet": "#f5c2e7",
                    "crimson": "#f38ba8",
                    "rainbow": "#f5c2e7"
                }
                custom_color = node.color.lower()
                if custom_color in color_map:
                    outline_color = color_map[custom_color]
                elif custom_color.startswith("#"):
                    outline_color = node.color
                else:
                    basic_colors = {
                        "white", "black", "red", "green", "blue",
                        "yellow", "cyan", "magenta"
                    }
                    if custom_color in basic_colors:
                        outline_color = node.color

            # Check capacity violation
            drones_here = node_occupancy.get(name, [])
            if (not node.is_start and not node.is_end and
                    len(drones_here) > node.max_drones):
                fill_color = "#582232"
                outline_color = "#f38ba8"

            # Draw node circle
            canvas.create_oval(
                cx - node_radius, cy - node_radius,
                cx + node_radius, cy + node_radius,
                fill=fill_color, outline=outline_color, width=3,
                tags=f"node-{name}"
            )

            # Label of the node
            label = name
            if node.is_start:
                label += "\n[Start]"
            elif node.is_end:
                label += "\n[Goal]"
            elif not node.is_start and not node.is_end and node.max_drones > 1:
                label += f"\n[{len(drones_here)}/{node.max_drones}]"
            elif len(drones_here) > 0:
                label += f"\n[{len(drones_here)}]"

            canvas.create_text(
                cx, cy, text=label, fill="#cdd6f4",
                font=("Inter", 8, "bold"), justify=tk.CENTER
            )

            # Draw Drones inside the node
            if drones_here:
                self.draw_drones_at_node(
                    canvas, cx, cy, node_radius, drones_here
                )

        # Draw In-Flight Drones on Edges
        for edge_key, drones in edge_occupancy.items():
            u, v = edge_key
            if u not in self.node_positions or v not in self.node_positions:
                continue
            x1, y1 = self.node_positions[u]
            x2, y2 = self.node_positions[v]

            # Midpoint position for in-flight representation
            self.draw_drones_on_edge(canvas, x1, y1, x2, y2, drones)

        # Re-apply current zoom/scale
        canvas.scale(
            "all", 0, 0,
            self.canvas_container.scale, self.canvas_container.scale
        )

    def draw_drones_at_node(
        self, canvas: tk.Canvas, cx: float, cy: float,
        radius: float, drones: List[str]
    ) -> None:
        """Arranges and draws drones inside or orbiting a node circle.

        Args:
            canvas: The Tkinter Canvas.
            cx: Canvas X coordinate of the node.
            cy: Canvas Y coordinate of the node.
            radius: Radius of the node.
            drones: List of drone ID strings at this node.
        """
        count = len(drones)
        drone_r = 6

        if count == 1:
            # Draw single drone offset from center to not obscure text
            canvas.create_oval(
                cx - drone_r, cy - radius - 10 - drone_r,
                cx + drone_r, cy - radius - 10 + drone_r,
                fill="#f9e2af", outline="#11111b", width=1
            )
            canvas.create_text(
                cx, cy - radius - 10, text=drones[0], fill="#11111b",
                font=("Inter", 6, "bold")
            )
        else:
            # Draw multiple drones orbiting the node
            for i, drone in enumerate(drones):
                angle = i * (2 * math.pi / count)
                dist = radius + 12
                dx = cx + dist * math.cos(angle)
                dy = cy + dist * math.sin(angle)

                canvas.create_oval(
                    dx - drone_r, dy - drone_r,
                    dx + drone_r, dy + drone_r,
                    fill="#f9e2af", outline="#11111b", width=1
                )
                canvas.create_text(
                    dx, dy, text=drone, fill="#11111b",
                    font=("Inter", 6, "bold")
                )

    def draw_drones_on_edge(
        self, canvas: tk.Canvas, x1: float, y1: float,
        x2: float, y2: float, drones: List[str]
    ) -> None:
        """Draws drones positioned along a connection line for transit state.

        Args:
            canvas: The Tkinter Canvas.
            x1: Canvas X coordinate of node 1.
            y1: Canvas Y coordinate of node 1.
            x2: Canvas X coordinate of node 2.
            y2: Canvas Y coordinate of node 2.
            drones: List of drone ID strings.
        """
        count = len(drones)
        drone_r = 7

        for i, drone in enumerate(drones):
            # Line interpolation: spread them out slightly around the midpoint
            t = 0.5
            if count > 1:
                t = 0.3 + (i / (count - 1)) * 0.4

            mx = x1 + (x2 - x1) * t
            my = y1 + (y2 - y1) * t

            canvas.create_oval(
                mx - drone_r, my - drone_r,
                mx + drone_r, my + drone_r,
                fill="#fab387", outline="#11111b", width=1
            )
            canvas.create_text(
                mx, my, text=drone, fill="#11111b",
                font=("Inter", 6, "bold")
            )

    def update_stats(self) -> None:
        """Updates the statistics sidebar panel with current simulation
        info."""
        self.stats_text.configure(state=tk.NORMAL)
        self.stats_text.delete("1.0", tk.END)

        if not self.graph:
            self.stats_text.insert(tk.END, "No map loaded.\n")
            self.stats_text.configure(state=tk.DISABLED)
            return

        map_name = os.path.basename(self.map_filepath)
        num_nodes = len(self.graph.nodes)
        num_edges = len(self.graph.edges)
        num_drones = len(self.graph.drones)

        stats = f"Map: {map_name}\n"
        stats += f"Nodes (Hubs): {num_nodes}\n"
        stats += f"Edges (Links): {num_edges}\n"
        stats += f"Total Drones: {num_drones}\n"
        stats += "-----------------------\n"

        if self.log_turns:
            current_positions = self.get_current_positions()
            end_node_name = (
                self.graph.endNode.name if self.graph.endNode else ""
            )

            # Count drones at destination
            at_dest = sum(
                1 for pos in current_positions.values() if pos == end_node_name
            )
            in_transit = num_drones - at_dest

            stats += f"Turn: {self.current_turn} / {len(self.log_turns)}\n"
            stats += f"At Destination: {at_dest}\n"
            stats += f"In Flight/Transit: {in_transit}\n"
            stats += "-----------------------\n"
            stats += "Node Occupancies:\n"

            # Show occupancy of nodes (excluding start/end)
            for name, node in self.graph.nodes.items():
                if node.is_start or node.is_end:
                    continue
                drones_here = sum(
                    1 for pos in current_positions.values() if pos == name
                )
                if drones_here > 0 or node.max_drones > 1:
                    status = ""
                    if drones_here > node.max_drones:
                        status = " ⚠️ OVER CAPACITY"
                    elif drones_here == node.max_drones:
                        status = " [FULL]"
                    stats += (
                        f"  {name}: {drones_here}/{node.max_drones}{status}\n"
                    )
        else:
            stats += "No simulation loaded.\n"
            stats += "Load a log file or click\n"
            stats += "'Generate Simple Simulation'\n"

        self.stats_text.insert(tk.END, stats)
        self.stats_text.configure(state=tk.DISABLED)

    def generate_dot_representation(self) -> str:
        """Generates a DOT graph representation with styling matching the turn.

        Returns:
            A string containing the Graphviz DOT code.
        """
        if not self.graph:
            return ""

        current_positions = self.get_current_positions()

        # Group drones by positions
        pos_drones: Dict[str, List[str]] = {}
        for drone, pos in current_positions.items():
            if pos not in pos_drones:
                pos_drones[pos] = []
            pos_drones[pos].append(drone)

        dot = "graph G {\n"
        dot += '    bgcolor="#11111b";\n'
        dot += '    pad="0.3";\n'
        dot += (
            '    edge [color="#585b70", fontcolor="#cdd6f4", '
            'fontname="Courier", fontsize="10"];\n'
        )
        dot += (
            '    node [fontcolor="#cdd6f4", fontname="Courier", '
            'style="filled", penwidth="2.5", shape="circle", '
            'width="1.2", fixedsize="false"];\n'
        )

        for name, node in self.graph.nodes.items():
            fillcolor = "#1e1e2e"
            color = "#89b4fa"

            if node.is_start:
                fillcolor = "#2e3f2f"
                color = "#a6e3a1"
            elif node.is_end:
                fillcolor = "#4c323c"
                color = "#f38ba8"
            elif node.zone == "restricted":
                fillcolor = "#453128"
                color = "#fab387"
            elif node.zone == "priority":
                fillcolor = "#392c47"
                color = "#cba6f7"
            elif node.zone == "blocked":
                fillcolor = "#313244"
                color = "#585b70"

            drones_here = pos_drones.get(name, [])
            label = name
            if drones_here:
                label += f"\\n[{','.join(drones_here)}]"

            # Over capacity alert style
            if (not node.is_start and not node.is_end and
                    len(drones_here) > node.max_drones):
                fillcolor = "#582232"
                color = "#f38ba8"

            dot += (
                f'    {name} [label="{label}", '
                f'fillcolor="{fillcolor}", color="{color}"];\n'
            )

        for edge in self.graph.edges:
            u, v = edge.start, edge.end

            # Check for in-flight drones on this edge
            # Either named "u-v" or "v-u"
            key1 = f"{u}-{v}"
            key2 = f"{v}-{u}"
            drones_on_edge = (
                pos_drones.get(key1, []) + pos_drones.get(key2, [])
            )

            color = "#585b70"
            width = "1.5"
            label_attr = ""

            if drones_on_edge:
                color = "#fab387"  # orange highlight
                width = "3.5"
                label_attr = f', label=" {",".join(drones_on_edge)}"'
            elif edge.capacity > 1:
                label_attr = f', label="c:{edge.capacity}"'

            dot += (
                f'    {u} -- {v} [color="{color}", '
                f'penwidth="{width}"{label_attr}];\n'
            )

        dot += "}\n"
        return dot

    def update_graphviz_view(self) -> None:
        """Renders the DOT graph via Graphviz and displays the PNG."""
        if not self.graph:
            return

        # Write DOT file
        dot_content = self.generate_dot_representation()
        dot_file = "temp_graph.dot"
        png_file = "temp_graph.png"

        try:
            with open(dot_file, "w") as f:
                f.write(dot_content)

            # Use dot layout (provides clean hierarchical representation)
            subprocess.run(
                ["dot", "-Tpng", dot_file, "-o", png_file],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=True
            )

            # Load image and render on the graphviz canvas
            img = Image.open(png_file)
            self.gv_image = ImageTk.PhotoImage(img)

            # Configure canvas scrollregion
            self.gv_canvas.delete("all")
            self.gv_canvas.create_image(
                10, 10, anchor=tk.NW, image=self.gv_image
            )
            self.gv_canvas.config(
                scrollregion=(0, 0, img.width + 20, img.height + 20)
            )

            # Cleanup temp files
            if os.path.exists(dot_file):
                os.remove(dot_file)
            if os.path.exists(png_file):
                os.remove(png_file)

        except Exception as e:
            # Fallback if dot execution fails
            self.gv_canvas.delete("all")
            self.gv_canvas.create_text(
                100, 50,
                text=(
                    f"Graphviz Render Error:\n{e}\n\n"
                    "Make sure 'dot' is installed and in path."
                ),
                fill="#f38ba8", font=("Inter", 10), anchor=tk.NW
            )

    def on_tab_changed(self, event: tk.Event) -> None:
        """Triggers Graphviz re-rendering only when Graphviz tab is active.

        Args:
            event: The Tkinter event object.
        """
        selected_tab = self.notebook.index(self.notebook.select())
        if selected_tab == 1:  # Graphviz View Tab
            self.update_graphviz_view()

    def toggle_play(self) -> None:
        """Toggles automatic simulation playback play/pause state."""
        if not self.log_turns:
            messagebox.showwarning(
                "Warning", "Please load a simulation log first!"
            )
            return

        self.is_playing = not self.is_playing
        if self.is_playing:
            self.btn_play.configure(text="⏸ Pause")
            self.run_playback_loop()
        else:
            self.btn_play.configure(text="▶ Play")
            if self.timer_id:
                self.root.after_cancel(self.timer_id)
                self.timer_id = None

    def run_playback_loop(self) -> None:
        """Continuously steps forward until the end or paused."""
        if not self.is_playing:
            return

        if self.current_turn < len(self.log_turns):
            self.step_forward()
            self.timer_id = self.root.after(
                self.playback_speed, self.run_playback_loop
            )
        else:
            self.is_playing = False
            self.btn_play.configure(text="▶ Play")
            self.timer_id = None

    def step_forward(self) -> None:
        """Advances the simulation by one turn."""
        if not self.log_turns:
            return
        if self.current_turn < len(self.log_turns):
            self.current_turn += 1
            self.time_slider.set(self.current_turn)
            self.update_turn_display()

    def step_back(self) -> None:
        """Steps back the simulation by one turn."""
        if not self.log_turns:
            return
        if self.current_turn > 0:
            self.current_turn -= 1
            self.time_slider.set(self.current_turn)
            self.update_turn_display()

    def go_to_start(self) -> None:
        """Resets simulation play to the beginning (Turn 0)."""
        if not self.log_turns:
            return
        self.current_turn = 0
        self.time_slider.set(0)
        self.update_turn_display()

    def go_to_end(self) -> None:
        """Advances simulation to the final turn."""
        if not self.log_turns:
            return
        self.current_turn = len(self.log_turns)
        self.time_slider.set(self.current_turn)
        self.update_turn_display()

    def change_speed(self, val: str) -> None:
        """Handles speed slider changes.

        Args:
            val: Float value as string from Scale widget.
        """
        self.playback_speed = int(float(val))

    def on_timeline_slide(self, val: str) -> None:
        """Handles timeline slider scrubbing.

        Args:
            val: Float value as string from Scale widget.
        """
        target_turn = int(float(val))
        if target_turn != self.current_turn:
            self.current_turn = target_turn
            self.update_turn_display()

    def update_turn_display(self) -> None:
        """Triggers redraws and updates stats for the current turn."""
        self.draw_interactive_graph()
        self.update_stats()
        # If Graphviz tab is currently visible, update it too
        selected_tab = self.notebook.index(self.notebook.select())
        if selected_tab == 1:
            self.update_graphviz_view()


def main(initial_map: Optional[str] = None) -> None:
    """Launches the Tkinter application."""
    root = tk.Tk()
    DroneVisualizerApp(root, initial_map=initial_map)
    root.mainloop()


if __name__ == "__main__":
    main()
