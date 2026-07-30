import os
import sys

from graph import Graph
from parser import Parser


def main() -> None:
    """Entry point for the Fly-in drone simulation."""
    if "--legacy-gui" in sys.argv:
        # Load the original graphical interface on demand.
        import gui

        # Find if a map file is passed along with the UI flag.
        map_file = None
        for arg in sys.argv[1:]:
            if arg != "--legacy-gui":
                map_file = arg
                break
        gui.main(initial_map=map_file)
    elif "--gui" in sys.argv or "--tui" in sys.argv:
        # Use the lightweight terminal interface for interactive use.
        import tui

        map_file = None
        for arg in sys.argv[1:]:
            if arg not in {"--gui", "--tui"}:
                map_file = arg
                break
        tui.main(initial_map=map_file)
    else:
        # Default behavior: run the solver and output the turns
        map_file = "./maps/challenger/01_the_impossible_dream.txt"
        if len(sys.argv) > 1:
            map_file = sys.argv[1]

        graph: Graph = Graph()
        parser: Parser = Parser()
        parser.parseFile(graph, map_file)

        # Route the drones using Google OR-Tools
        from solver import OrToolsSolver, SimulationEngine

        solver = OrToolsSolver(graph)
        paths = solver.solve()

        # Generate the simulation output
        engine = SimulationEngine(graph, paths)

        # Print map_name header followed by turn-by-turn logs
        map_name = os.path.basename(map_file)
        print(map_name)
        engine.print_output()


if __name__ == "__main__":
    main()
