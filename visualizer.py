"""Pygame-based graphical visualizer for Fly-in drone routing simulation."""

import math
import os
import sys
from typing import Dict, Final, List, Optional, Tuple

import pygame

from graph import Graph
from parser import Parser
from solver import DroneRouter, SimulationEngine

# Color Palette (Dark modern theme)
COLOR_BG: Final[Tuple[int, int, int]] = (15, 23, 42)            # Slate 900
COLOR_PANEL_BG: Final[Tuple[int, int, int]] = (30, 41, 59)      # Slate 800
COLOR_PANEL_BORDER: Final[Tuple[int, int, int]] = (51, 65, 85)  # Slate 700
COLOR_TEXT: Final[Tuple[int, int, int]] = (241, 245, 249)       # Slate 100
COLOR_TEXT_MUTED: Final[Tuple[int, int, int]] = (148, 163, 184)  # Slate 400
COLOR_EDGE: Final[Tuple[int, int, int]] = (71, 85, 105)          # Slate 600
COLOR_EDGE_ACTIVE: Final[Tuple[int, int, int]] = (56, 189, 248)  # Sky 400

# Zone Type Colors
ZONE_COLORS: Final[Dict[str, Tuple[int, int, int]]] = {
    "normal": (59, 130, 246),      # Blue
    "start": (34, 197, 94),        # Green
    "end": (234, 179, 8),          # Amber / Gold
    "restricted": (239, 68, 68),   # Red / Orange
    "priority": (168, 85, 247),    # Purple / Magenta
    "blocked": (100, 116, 139),    # Slate / Gray
}

# Drone Theme Colors
DRONE_COLOR: Final[Tuple[int, int, int]] = (14, 165, 233)     # Cyan / Sky
DRONE_DELIVERED: Final[Tuple[int, int, int]] = (34, 197, 94)  # Emerald green


class VisualizerButton:
    """Represents a clickable UI button."""

    def __init__(self, rect: pygame.Rect, text: str, action_tag: str) -> None:
        """Initializes a button.

        Args:
            rect: Pygame bounding rectangle.
            text: Button label.
            action_tag: Identifier tag for button action.
        """
        self.rect = rect
        self.text = text
        self.action_tag = action_tag
        self.is_hovered = False

    def check_hover(self, pos: Tuple[int, int]) -> None:
        """Updates hover state based on mouse position.

        Args:
            pos: Mouse (x, y) coordinates.
        """
        self.is_hovered = self.rect.collidepoint(pos)

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        """Draws the button onto the surface.

        Args:
            surface: Pygame drawing surface.
            font: Pygame font for rendering text.
        """
        bg_color = (51, 65, 85) if self.is_hovered else (30, 41, 59)
        border_c = (94, 234, 212) if self.is_hovered else COLOR_PANEL_BORDER
        pygame.draw.rect(surface, bg_color, self.rect, border_radius=6)
        pygame.draw.rect(surface, border_c, self.rect, 2, border_radius=6)
        text_surf = font.render(self.text, True, COLOR_TEXT)
        text_rect = text_surf.get_rect(center=self.rect.center)
        surface.blit(text_surf, text_rect)


class PygameVisualizer:
    """Interactive graphical simulator for Fly-in drone paths using Pygame."""

    def __init__(self, map_file: str, width: int = 1200, height: int = 800) -> None:
        """Initializes the visualizer with map path and window geometry.

        Args:
            map_file: Path to the network map file.
            width: Initial window width in pixels.
            height: Initial window height in pixels.
        """
        self.map_file = map_file
        self.map_name = os.path.basename(map_file)
        self.width = width
        self.height = height

        # Graph and Simulation Data
        self.graph = Graph()
        parser = Parser()
        parser.parse_file(self.graph, self.map_file)

        router = DroneRouter(self.graph)
        self.paths = router.solve()
        self.engine = SimulationEngine(self.graph, self.paths)

        # Simulation Playback State
        self.total_turns = max(len(p) - 1 for p in self.paths) if self.paths else 0
        self.current_turn = 0
        self.is_playing = False
        self.play_speed = 1.0  # Turns per second
        self.time_accumulator = 0.0
        self.sub_turn_progress = 0.0  # 0.0 to 1.0 interpolation for smooth animation

        # Camera & Transform
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.is_panning = False
        self.pan_start = (0, 0)
        self.node_positions: Dict[str, Tuple[float, float]] = {}

        # Pygame Subsystem
        pygame.init()
        pygame.display.set_caption(f"Fly-in Simulation: {self.map_name}")
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()

        # Fonts
        self.font_title = pygame.font.SysFont("DejaVu Sans, Arial, Helvetica", 20, bold=True)
        self.font_main = pygame.font.SysFont("DejaVu Sans, Arial, Helvetica", 14)
        self.font_small = pygame.font.SysFont("DejaVu Sans, Arial, Helvetica", 11)

        self._init_node_positions()
        self.buttons: List[VisualizerButton] = []
        self._init_ui_buttons()

    def _init_node_positions(self) -> None:
        """Calculates initial normalized screen coordinates for all graph nodes."""
        if not self.graph.nodes:
            return

        min_x = min(node.x for node in self.graph.nodes.values())
        max_x = max(node.x for node in self.graph.nodes.values())
        min_y = min(node.y for node in self.graph.nodes.values())
        max_y = max(node.y for node in self.graph.nodes.values())

        span_x = max(1, max_x - min_x)
        span_y = max(1, max_y - min_y)

        margin_x = 120
        margin_top = 100
        margin_bottom = 120
        usable_w = self.width - 2 * margin_x
        usable_h = self.height - margin_top - margin_bottom

        scale_x = usable_w / span_x if span_x > 0 else 1.0
        scale_y = usable_h / span_y if span_y > 0 else 1.0
        scale = min(scale_x, scale_y, 160.0)

        center_data_x = (min_x + max_x) / 2.0
        center_data_y = (min_y + max_y) / 2.0

        canvas_center_x = self.width / 2.0
        canvas_center_y = margin_top + usable_h / 2.0

        for name, node in self.graph.nodes.items():
            screen_x = canvas_center_x + (node.x - center_data_x) * scale
            screen_y = canvas_center_y - (node.y - center_data_y) * scale
            self.node_positions[name] = (screen_x, screen_y)

    def _init_ui_buttons(self) -> None:
        """Creates interactive buttons in the bottom control panel."""
        self.buttons.clear()
        panel_y = self.height - 70
        btn_w, btn_h = 100, 36
        start_x = self.width // 2 - 270

        self.buttons.append(VisualizerButton(pygame.Rect(start_x, panel_y, btn_w, btn_h), "Reset (R)", "reset"))
        self.buttons.append(VisualizerButton(pygame.Rect(start_x + 110, panel_y, btn_w, btn_h), "Prev (<-)", "prev"))
        self.buttons.append(VisualizerButton(pygame.Rect(start_x + 220, panel_y, btn_w, btn_h), "Play/Pause", "play"))
        self.buttons.append(VisualizerButton(pygame.Rect(start_x + 330, panel_y, btn_w, btn_h), "Next (->)", "next"))
        speed_label = f"Speed: {self.play_speed}x"
        self.buttons.append(VisualizerButton(pygame.Rect(start_x + 440, panel_y, btn_w, btn_h), speed_label, "speed"))

    def _to_screen(self, pt: Tuple[float, float]) -> Tuple[int, int]:
        """Converts graph coordinates to screen coordinates taking pan and zoom into account.

        Args:
            pt: (x, y) point.

        Returns:
            Transformed (sx, sy) tuple.
        """
        center_x = self.width / 2.0
        center_y = self.height / 2.0
        sx = center_x + (pt[0] - center_x + self.pan_x) * self.zoom
        sy = center_y + (pt[1] - center_y + self.pan_y) * self.zoom
        return int(sx), int(sy)

    def _get_drone_position(self, drone_idx: int, turn_float: float) -> Tuple[float, float]:
        """Calculates interpolated (x, y) coordinate for a drone at fractional turn.

        Args:
            drone_idx: Zero-indexed drone index.
            turn_float: Continuous simulation turn number.

        Returns:
            (x, y) position on canvas.
        """
        path = self.paths[drone_idx]
        t0 = int(turn_float)
        t1 = min(len(path) - 1, t0 + 1)
        fraction = turn_float - t0

        state0 = path[min(t0, len(path) - 1)]
        state1 = path[t1]

        def resolve_coord(state: str) -> Tuple[float, float]:
            if state in self.node_positions:
                return self.node_positions[state]
            if "-" in state:
                parts = state.split("-", 1)
                p1 = self.node_positions.get(parts[0], (self.width / 2.0, self.height / 2.0))
                p2 = self.node_positions.get(parts[1], (self.width / 2.0, self.height / 2.0))
                return ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)
            return (self.width / 2.0, self.height / 2.0)

        pos0 = resolve_coord(state0)
        pos1 = resolve_coord(state1)

        # Smooth cubic ease
        smooth_frac = fraction * fraction * (3.0 - 2.0 * fraction)
        interp_x = pos0[0] + (pos1[0] - pos0[0]) * smooth_frac
        interp_y = pos0[1] + (pos1[1] - pos0[1]) * smooth_frac

        # Stagger overlapping drones at start hub or shared nodes
        offset_angle = (drone_idx * 137.5) * (math.pi / 180.0)
        radius = (drone_idx % 4) * 6.0
        return (interp_x + math.cos(offset_angle) * radius, interp_y + math.sin(offset_angle) * radius)

    def handle_event(self, event: pygame.event.Event) -> bool:
        """Processes user input events (keyboard, mouse, window).

        Args:
            event: Pygame event.

        Returns:
            False if application should exit, True otherwise.
        """
        if event.type == pygame.QUIT:
            return False

        if event.type == pygame.VIDEORESIZE:
            self.width = event.w
            self.height = event.h
            self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
            self._init_node_positions()
            self._init_ui_buttons()
            return True

        if event.type == pygame.KEYDOWN:
            if event.key in (pygame.K_ESCAPE, pygame.K_q):
                return False
            if event.key == pygame.K_SPACE:
                self.is_playing = not self.is_playing
            elif event.key in (pygame.K_RIGHT, pygame.K_l):
                self.step_forward()
            elif event.key in (pygame.K_LEFT, pygame.K_h):
                self.step_backward()
            elif event.key == pygame.K_r:
                self.reset()
            elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):
                self.increase_speed()
            elif event.key == pygame.K_MINUS:
                self.decrease_speed()
            elif event.key == pygame.K_f:
                self.fit_view()

        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = event.pos
            if event.button == 1:
                clicked_button = False
                for btn in self.buttons:
                    if btn.rect.collidepoint(mouse_pos):
                        self._trigger_button_action(btn.action_tag)
                        clicked_button = True
                        break

                scrubber_rect = pygame.Rect(100, self.height - 110, self.width - 200, 16)
                if scrubber_rect.collidepoint(mouse_pos) and self.total_turns > 0:
                    fraction = (mouse_pos[0] - scrubber_rect.x) / scrubber_rect.width
                    self.current_turn = max(0, min(self.total_turns, int(fraction * self.total_turns)))
                    self.sub_turn_progress = 0.0
                    clicked_button = True

                if not clicked_button:
                    self.is_panning = True
                    self.pan_start = mouse_pos

            elif event.button == 4:
                self.zoom = min(4.0, self.zoom * 1.1)
            elif event.button == 5:
                self.zoom = max(0.2, self.zoom / 1.1)

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self.is_panning = False

        elif event.type == pygame.MOUSEMOTION:
            if self.is_panning:
                dx = (event.pos[0] - self.pan_start[0]) / self.zoom
                dy = (event.pos[1] - self.pan_start[1]) / self.zoom
                self.pan_x += dx
                self.pan_y += dy
                self.pan_start = event.pos

            for btn in self.buttons:
                btn.check_hover(event.pos)

        return True

    def _trigger_button_action(self, action_tag: str) -> None:
        """Executes the action associated with a button tag.

        Args:
            action_tag: Action identifier.
        """
        if action_tag == "reset":
            self.reset()
        elif action_tag == "prev":
            self.step_backward()
        elif action_tag == "play":
            self.is_playing = not self.is_playing
        elif action_tag == "next":
            self.step_forward()
        elif action_tag == "speed":
            self.increase_speed()

    def reset(self) -> None:
        """Resets the simulation turn to 0."""
        self.current_turn = 0
        self.sub_turn_progress = 0.0
        self.is_playing = False

    def step_forward(self) -> None:
        """Advances the simulation by one turn."""
        if self.current_turn < self.total_turns:
            self.current_turn += 1
            self.sub_turn_progress = 0.0

    def step_backward(self) -> None:
        """Rewinds the simulation by one turn."""
        if self.current_turn > 0:
            self.current_turn -= 1
            self.sub_turn_progress = 0.0

    def increase_speed(self) -> None:
        """Increases playback speed."""
        speeds = [0.5, 1.0, 2.0, 4.0, 8.0]
        idx = speeds.index(self.play_speed) if self.play_speed in speeds else 1
        self.play_speed = speeds[(idx + 1) % len(speeds)]
        for btn in self.buttons:
            if btn.action_tag == "speed":
                btn.text = f"Speed: {self.play_speed}x"

    def decrease_speed(self) -> None:
        """Decreases playback speed."""
        speeds = [0.5, 1.0, 2.0, 4.0, 8.0]
        idx = speeds.index(self.play_speed) if self.play_speed in speeds else 1
        self.play_speed = speeds[(idx - 1 + len(speeds)) % len(speeds)]
        for btn in self.buttons:
            if btn.action_tag == "speed":
                btn.text = f"Speed: {self.play_speed}x"

    def fit_view(self) -> None:
        """Resets zoom and pan to centered fit."""
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0

    def update(self, dt: float) -> None:
        """Updates animation time and simulation progression.

        Args:
            dt: Elapsed delta time in seconds.
        """
        if self.is_playing and self.current_turn < self.total_turns:
            self.sub_turn_progress += dt * self.play_speed
            if self.sub_turn_progress >= 1.0:
                self.current_turn += int(self.sub_turn_progress)
                self.sub_turn_progress %= 1.0
                if self.current_turn >= self.total_turns:
                    self.current_turn = self.total_turns
                    self.sub_turn_progress = 0.0
                    self.is_playing = False

    def draw(self) -> None:
        """Renders all elements: background, grid, connections, zones, drones, HUD."""
        self.screen.fill(COLOR_BG)

        # Draw Connections / Edges
        for edge in self.graph.edges:
            p1_raw = self.node_positions.get(edge.start)
            p2_raw = self.node_positions.get(edge.end)
            if p1_raw and p2_raw:
                p1 = self._to_screen(p1_raw)
                p2 = self._to_screen(p2_raw)

                thickness = min(6, max(2, edge.capacity * 2))
                pygame.draw.line(self.screen, COLOR_EDGE, p1, p2, thickness)

                if edge.capacity > 1:
                    mid_x = (p1[0] + p2[0]) // 2
                    mid_y = (p1[1] + p2[1]) // 2
                    cap_surf = self.font_small.render(f"cap:{edge.capacity}", True, COLOR_TEXT_MUTED)
                    self.screen.blit(cap_surf, (mid_x + 4, mid_y - 12))

        # Draw Nodes / Zones
        for name, node in self.graph.nodes.items():
            pos_raw = self.node_positions.get(name)
            if not pos_raw:
                continue
            center = self._to_screen(pos_raw)
            radius = int(22 * self.zoom)
            radius = max(10, min(60, radius))

            if node.is_start:
                node_color = ZONE_COLORS["start"]
            elif node.is_end:
                node_color = ZONE_COLORS["end"]
            elif node.is_restricted():
                node_color = ZONE_COLORS["restricted"]
            elif node.is_priority():
                node_color = ZONE_COLORS["priority"]
            elif node.is_blocked():
                node_color = ZONE_COLORS["blocked"]
            else:
                node_color = ZONE_COLORS["normal"]

            pygame.draw.circle(self.screen, node_color, center, radius)
            pygame.draw.circle(self.screen, COLOR_TEXT, center, radius, max(1, int(2 * self.zoom)))

            label_text = name
            if node.is_restricted():
                label_text += " [R:2t]"
            elif node.is_priority():
                label_text += " [P]"
            elif node.is_blocked():
                label_text += " [X]"

            label_surf = self.font_small.render(label_text, True, COLOR_TEXT)
            label_rect = label_surf.get_rect(center=(center[0], center[1] + radius + 12))
            self.screen.blit(label_surf, label_rect)

            if not node.is_start and not node.is_end and not node.is_blocked():
                cap_text = f"max:{node.max_drones}"
                cap_surf = self.font_small.render(cap_text, True, COLOR_TEXT_MUTED)
                cap_rect = cap_surf.get_rect(center=(center[0], center[1] - radius - 10))
                self.screen.blit(cap_surf, cap_rect)

        # Draw Drones
        turn_float = self.current_turn + (self.sub_turn_progress if self.is_playing else 0.0)
        delivered_count = 0

        for d_idx, path in enumerate(self.paths):
            pos_raw = self._get_drone_position(d_idx, turn_float)
            d_center = self._to_screen(pos_raw)
            d_radius = max(6, int(10 * self.zoom))

            current_state = path[min(int(turn_float), len(path) - 1)]
            is_delivered = (current_state == self.graph.end_node.name if self.graph.end_node else False)
            if is_delivered:
                delivered_count += 1

            drone_c = DRONE_DELIVERED if is_delivered else DRONE_COLOR
            pygame.draw.circle(self.screen, drone_c, d_center, d_radius)
            pygame.draw.circle(self.screen, (255, 255, 255), d_center, d_radius, 1)

            drone_label = f"D{d_idx + 1}"
            d_surf = self.font_small.render(drone_label, True, (255, 255, 255))
            d_rect = d_surf.get_rect(center=d_center)
            self.screen.blit(d_surf, d_rect)

        self._draw_header(delivered_count)
        self._draw_controls()
        pygame.display.flip()

    def _draw_header(self, delivered_count: int) -> None:
        """Draws top information banner and telemetry.

        Args:
            delivered_count: Number of drones at destination.
        """
        header_rect = pygame.Rect(0, 0, self.width, 60)
        pygame.draw.rect(self.screen, COLOR_PANEL_BG, header_rect)
        pygame.draw.line(self.screen, COLOR_PANEL_BORDER, (0, 60), (self.width, 60), 2)

        title_surf = self.font_title.render(f"Fly-in: {self.map_name}", True, COLOR_TEXT)
        self.screen.blit(title_surf, (20, 16))

        status_str = "PLAYING" if self.is_playing else (
            "COMPLETED" if self.current_turn >= self.total_turns else "PAUSED"
        )
        telemetry = (
            f"Turn: {self.current_turn}/{self.total_turns}   |   "
            f"Drones: {delivered_count}/{len(self.paths)} Delivered   |   "
            f"Status: {status_str}"
        )
        info_surf = self.font_main.render(telemetry, True, COLOR_TEXT)
        self.screen.blit(info_surf, (self.width - info_surf.get_width() - 30, 20))

    def _draw_controls(self) -> None:
        """Draws bottom scrubber bar, control buttons, and legend."""
        scrubber_bg = pygame.Rect(100, self.height - 110, self.width - 200, 12)
        pygame.draw.rect(self.screen, COLOR_PANEL_BG, scrubber_bg, border_radius=6)
        pygame.draw.rect(self.screen, COLOR_PANEL_BORDER, scrubber_bg, 1, border_radius=6)

        if self.total_turns > 0:
            fill_w = int((self.current_turn / self.total_turns) * scrubber_bg.width)
            fill_rect = pygame.Rect(scrubber_bg.x, scrubber_bg.y, fill_w, scrubber_bg.height)
            pygame.draw.rect(self.screen, (56, 189, 248), fill_rect, border_radius=6)

        for btn in self.buttons:
            btn.draw(self.screen, self.font_main)

    def run(self) -> None:
        """Main visualizer event and render loop."""
        running = True
        while running:
            dt = self.clock.tick(60) / 1000.0
            for event in pygame.event.get():
                if not self.handle_event(event):
                    running = False
                    break

            self.update(dt)
            self.draw()

        pygame.quit()


def main(initial_map: Optional[str] = None) -> None:
    """Entry point for the Pygame visualizer.

    Args:
        initial_map: Optional map file path to display.
    """
    map_file = initial_map or "./maps/easy/01_linear_path.txt"
    if len(sys.argv) > 1 and not initial_map:
        for arg in sys.argv[1:]:
            if not arg.startswith("-"):
                map_file = arg
                break

    try:
        app = PygameVisualizer(map_file)
        app.run()
    except Exception as err:
        print(f"Visualizer Error: {err}", file=sys.stderr)


if __name__ == "__main__":
    main()
