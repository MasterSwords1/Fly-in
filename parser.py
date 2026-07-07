from graph import Graph
from node import Node
from drone import Drone
from edge import Edge
import re


class Parser():

    def __init__(self) -> None:
        self.patterns: list[str] = [
            r"^nb_drones: (?P<nb_drones>\d{1,})$",
            r"^start_hub: (?P<hub_name>\w+) (?P<x>\d{0,}) (?P<y>\d{0,})(?P<metadata>\s+\[(\s?\w+\=\w+\s?)?\])?$",
            r"^end_hub: (?P<hub_name>\w+) (?P<x>\d{0,}) (?P<y>\d{0,})(?P<metadata> \[(\s?\w+\=\w+\s?)?\])?$",
            r"^hub: (?P<hub_name>\w+) (?P<x>\d{0,}) (?P<y>\d{0,})(?P<metadata> \[(\s?\w+\=\w+\s?)?\])?$",
            r"^connection: (?P<hubs>\w+\-\w+)(?P<metadata> \[(\s?\w+\=\w+\s?)?\])?$"
        ]
        self.init_drones: bool = False
        self.zoneTypes: list[str] = ["normal", "restricted", "priority", "blocked"]
        self.drones: list[Drone] = []

    def createDrones(self, line: str) -> None:
        if self.init_drones:
            raise Exception("Drones number redifinition")
        num: int = int(re.match(self.patterns[0], line).group(1))
        for i in range(num):
            self.drones.append(Drone(i, None))
    
    def extractMetadata(self, line: str, index: int, g_id: int) -> dict[str, str]:
        metadata: dict[str, str] = {}
        data: list[str] = re.match(self.patterns[index], line).group(g_id).replace("[", "").replace("]", "").strip().split(" ")
        for d in data:
            metadata[d.split("=")[0]] = d.split("=")[1]
        return metadata

    def createHub(self, line: str, index: int) -> Node:
        match: re.Match = re.match(self.patterns[index], line)
        hub: Node = Node(match.group(1), int(match.group(2)), int(match.group(3)))
        return hub
    
    def addMetadataToHub(self, hub: Node, metadata: dict[str, str]) -> None:
        hub.zone = metadata.get("zone", "normal")
        if not hub.zone in self.zoneTypes:
            raise Exception(f"Invalid zone type: {hub.zone}")
        if hub.zone == "blocked":
            hub.cost = -999
        elif hub.zone == "restricted":
            hub.cost = 2
        hub.color = metadata["color"] if metadata["color"] else ""
        hub.max_drones = int(metadata.get("max_drones", "1"))
        if hub.max_drones < 1:
            raise Exception(f"Invalid data: {hub.max_drones}")
        return


    def parseLine(self, graph: Graph, line: str) -> None:
        if re.search(self.patterns[0], line):
            self.createDrones(line)
            graph.drones = self.drones
            return
        if re.search(self.patterns[1], line):
            match: re.Match = re.match(self.patterns[1], line)
            start: Node = self.createHub(line, 1)
            if match.group(4):
                metadata: dict[str, str] = self.extractMetadata(line, 1, 4)
                start.color = metadata.get("color", "")
            start.is_start = True
            start.max_drones = -1
            graph.addStart(start)
            return
        if re.search(self.patterns[2], line):
            match: re.Match = re.match(self.patterns[2], line)
            end: Node = self.createHub(line, 2)
            if match.group(4):
                metadata: dict[str, str] = self.extractMetadata(line, 2, 4)
                end.color = metadata.get("color", "")
            end.is_end = True
            end.max_drones = -1
            graph.addEnd(end)
            return
        if re.search(self.patterns[3], line):
            match: re.Match = re.match(self.patterns[3], line)
            hub: Node = self.createHub(line, 3)
            if match.group(4):
                metadata: dict[str, str] = self.extractMetadata(line, 3, 4)
                self.addMetadataToHub(hub, metadata)
            graph.addNode(hub)
            return
        if re.search(self.patterns[4], line):
            match: re.Match = re.match(self.patterns[4], line)
            edge: Edge = Edge(match.group(1).split("-")[0], match.group(1).split("-")[1])
            if match.group(2):
                metadata: dict[str, str] = self.extractMetadata(line, 4, 2)
                edge.capacity = int(metadata.get("max_link_capacity", "1"))
                if edge.capacity < 1:
                    raise Exception(f"Invalid metadata in line: {line}")
            graph.addEdge(edge)
            return



    def parseFile(self, graph: Graph, filename: str) -> None:
        try:
            with open(filename, "r") as file:
                i: int = 0
                for line in file:
                    line = line.strip()
                    if line.startswith("#") or line == "":
                        continue
                    if i == 0 and not line.startswith("nb_drone"):
                        raise Exception("file must start with nb_drones: <number_of_drones>")
                    self.parseLine(graph, line)
                    i += 1

        except Exception as e:
            print(f"Error: {e}")
