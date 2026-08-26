"""Main entry point for Fly-in drone routing simulation and visualizer."""

import os
import sys
from typing import List, Optional

from graph import Graph
from parser import Parser, ParsingError
from solver import DroneRouter, RoutingError, SimulationEngine


def run_simulation(map_file: str) -> int:
    """Executes the drone pathfinding simulation and prints step-by-step logs.

    Args:
        map_file: Path to the target map file.

    Returns:
        0 on success, non-zero error code on failure.
    """
    if not os.path.exists(map_file):
        print(f"Error: Map file '{map_file}' not found", file=sys.stderr)
        return 1

    graph = Graph()
    parser = Parser()

    try:
        parser.parse_file(graph, map_file)
    except ParsingError as err:
        print(f"Parsing Error: {err}", file=sys.stderr)
        return 1
    except Exception as err:
        print(f"Unexpected Parser Error: {err}", file=sys.stderr)
        return 1

    try:
        router = DroneRouter(graph)
        paths = router.solve()
        engine = SimulationEngine(graph, paths)

        # Output the map filename header followed by step-by-step turns
        map_name = os.path.basename(map_file)
        print(map_name)
        engine.print_output()
        return 0
    except RoutingError as err:
        print(f"Routing Error: {err}", file=sys.stderr)
        return 1
    except Exception as err:
        print(f"Simulation Error: {err}", file=sys.stderr)
        return 1


def run_gui(map_file: Optional[str]) -> int:
    """Launches the Pygame interactive graphical simulation visualizer.

    Args:
        map_file: Path to the map file to visualize.

    Returns:
        0 on success, non-zero error code on failure.
    """
    try:
        import visualizer
        target_map = map_file or "./maps/easy/01_linear_path.txt"
        visualizer.main(initial_map=target_map)
        return 0
    except Exception as err:
        print(f"Visualizer Error: {err}", file=sys.stderr)
        return 1


def print_help() -> None:
    """Prints command line interface usage guidelines."""
    help_text = (
        "Fly-in: Autonomous Drone Fleet Pathfinding Simulation\n\n"
        "Usage:\n"
        "  python main.py <map_file>           Run CLI simulation and print turns\n"
        "  python main.py --gui <map_file>     Launch interactive Pygame visualizer\n"
        "  python main.py -g <map_file>        Short flag for Pygame visualizer\n"
        "  python main.py --help               Display this help message\n\n"
        "Examples:\n"
        "  python main.py maps/easy/01_linear_path.txt\n"
        "  python main.py --gui maps/hard/01_maze_nightmare.txt\n"
    )
    print(help_text)


def main(args: Optional[List[str]] = None) -> None:
    """Main execution function evaluating CLI arguments.

    Args:
        args: Optional list of command line arguments (defaults to sys.argv[1:]).
    """
    cli_args = args if args is not None else sys.argv[1:]

    if not cli_args or "--help" in cli_args or "-h" in cli_args:
        if not cli_args:
            # Default to challenger or easy if no argument provided
            default_map = "./maps/easy/01_linear_path.txt"
            exit_code = run_simulation(default_map)
            sys.exit(exit_code)
        print_help()
        sys.exit(0)

    # Check for GUI mode
    is_gui_mode = "--gui" in cli_args or "-g" in cli_args
    map_files = [arg for arg in cli_args if not arg.startswith("-")]

    target_map = map_files[0] if map_files else "./maps/easy/01_linear_path.txt"

    if is_gui_mode:
        code = run_gui(target_map)
    else:
        code = run_simulation(target_map)

    sys.exit(code)


if __name__ == "__main__":
    main()
