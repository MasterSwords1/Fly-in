from graph.graph import Graph, Node
import re
from enum import Enum



def parseLine(graph: Graph, line: str) -> None:
    patterns: list[str] = [
        r"^nb_drones: (?P<nb_drones>\d{1,})?$",
        r"^start_hub: (?p<hub_name>\w+) \d{0,} \d{0,}(?P<metadata> \[\w?\])?$",
        r"^end_hub: (?p<hub_name>\w+) \d{0,} \d{0,}(?P<metadata> \[\w?\])?$",
        r"^hub: (?p<hub_name>\w+) \d{0,} \d{0,}(?P<metadata> \[\w?\])?$",
        r"^connection: \w+\-\w+(?P<metadata> \[\w?\])?$"
    ]
    funcs = list[function] = [, ]
    match True:
        case line


def parseFile(graph: Graph, filename: str) -> None:
    try:
        with open(filename, "r") as file:
            i: int = 0
            for line in file:
                line.strip()
                if line.startswith("#") or line == "":
                    continue
                if i is 0 and not line.startswith("nb_drone"):
                    raise Exception("file must start with nb_drones: <number_of_drones>")
                parseLine(graph, line)

    except Exception as e:
        print(f"Error: {e}")