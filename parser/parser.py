from graph.graph import Graph
from graph.node import Node, ZoneType
from graph.drone import Drone
from graph.edge import Edge
import re
from enum import Enum


class Parser():

    patterns: list[str] = [
        r"^nb_drones: (?P<nb_drones>\d{1,})$",
        r"^start_hub: (?P<hub_name>\w+) \d{0,} \d{0,}(?P<metadata> \[(\s?\w+\=\w+\s?)?\])?$",
        r"^end_hub: (?P<hub_name>\w+) \d{0,} \d{0,}(?P<metadata> \[(\s?\w+\=\w+\s?)?\])?$",
        r"^hub: (?P<hub_name>\w+) \d{0,} \d{0,}(?P<metadata> \[(\s?\w+\=\w+\s?)?\])?$",
        r"^connection: (?P<hubs>\w+\-\w+)(?P<metadata> \[(\s?\w+\=\w+\s?)?\])?$"
    ]
    init_drones: bool = False
    drones: list[Drone] = []

    def createDrones(line: str) -> None:
        if Parser.init_drones:
            raise Exception("Drones number redifinition")
        num: int = int(re.match(Parser.patterns[0], line).group(0))
        for i in range(num):
            Parser.drones.append(Drone(i, None))
    
    def extractMetadata(line: str, index: int) -> dict[str, str]:
        metadata: dict[str, str] = {}
        data: list[str] = re.match(Parser.patterns[index], line).group(3).replace("[", "").replace("]", "").split(" ")
        for d in data:
            metadata[d.split("=")[0]] = d.split("=")[1]
        return metadata

    def createHub(line: str, index: int) -> Node:
        match: re.Match = re.match(Parser.patterns[index], line)
        hub: Node = Node(match.group(0), int(match.group(1)), int(match.group(2)))
        return hub
    
    def addMetadataToHub(hub: Node, metadata: dict[str, str]) -> None:
        hub.zone = metadata["zone"]
        if not hub.zone in ZoneType:
            raise Exception(f"Invalid zone type: {hub.zone}")
        if hub.zone == ZoneType.BLOCKED:
            hub.cost = -999
        elif hub.zone == ZoneType.RESTRICTED:
            hub.cost = 2
        hub.color = metadata["color"] if metadata["color"] else ""
        hub.max_drones = int(metadata["max_drones"] if metadata["max_drones"] else 1)
        if hub.max_drones < 1:
            raise Exception(f"Invalid data: {hub.max_drones}")
        return


    def parseLine(graph: Graph, line: str) -> None:
        if re.search(Parser.patterns[0], line):
            Parser.createDrones(line)
            return
        if re.search(Parser.patterns[1], line):
            start: Node = Parser.createHub(line, 1)
            if match.group(3):
                metadata: dict[str, str] = Parser.extractMetadata(line, 1)
                start.color = metadata["color"] if metadata["color"] else ""
            start.is_start = True
            start.max_drone = -1
            graph.addStart(start)
            return
        if re.search(Parser.patterns[2], line):
            end: Node = Parser.createHub(line, 2)
            if match.group(3):
                metadata: dict[str, str] = Parser.extractMetadata(line, 2)
                start.color = metadata["color"] if metadata["color"] else ""
            end.is_end = True
            end.max_drone = -1
            graph.addEnd(end)
            return
        if re.search(Parser.patterns[3], line):
            hub: Node = Parser.createHub(line, 3)
            if match.group(3):
                metadata: dict[str, str] = Parser.extractMetadata(line, 3)
                Parser.addMetadataToHub(hub, metadata)
            graph.addNode(hub)
            return
        if re.search(Parser.patterns[4], line):
            match: re.Match = re.match(Parser.patterns[4], line)
            edge: Edge = Edge(match.group(0).split("-")[0], match.group(0).split("-")[1])
            if match.group(3):
                metadata: dict[str, str] = Parser.extractMetadata(line, 4)
                edge.capacity = int(metadata["max_link_capacity"]) if int(metadata["max_link_capacity"]) else 1
                if edge.capacity < 1:
                    raise Exception(f"Invalid metadata in line: {line}")
            graph.addEdge(edge)
            return



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
                    Parser.parseLine(graph, line)

        except Exception as e:
            print(f"Error: {e}")
