import re
from graph import Graph
from node import Node
from drone import Drone
from edge import Edge


class Parser():

    def __init__(self) -> None:
        self.patterns: list[str] = [
            r"^nb_drones: (?P<nb_drones>\d+)$",
            r"^start_hub: (?P<hub_name>\w+) (?P<x>-?\d+) (?P<y>-?\d+)"
            r"(?P<metadata> \[(?:\s*\w+=\w+)*\s*\])?$",
            r"^end_hub: (?P<hub_name>\w+) (?P<x>-?\d+) (?P<y>-?\d+)"
            r"(?P<metadata> \[(?:\s*\w+=\w+)*\s*\])?$",
            r"^hub: (?P<hub_name>\w+) (?P<x>-?\d+) (?P<y>-?\d+)"
            r"(?P<metadata> \[(?:\s*\w+=\w+)*\s*\])?$",
            r"^connection: (?P<hubs>\w+\-\w+)"
            r"(?P<metadata> \[(?:\s*\w+=\w+)*\s*\])?$"
        ]
        self.init_drones: bool = False
        self.zoneTypes: list[str] = [
            "normal", "restricted", "priority", "blocked"
        ]
        self.drones: list[Drone] = []

    def createDrones(self, line: str) -> None:
        if self.init_drones:
            raise Exception("Drones number redifinition")
        match = re.match(self.patterns[0], line)
        if not match:
            raise Exception("Invalid drones number line")
        num: int = int(match.group(1))
        for i in range(num):
            self.drones.append(Drone(i, None))

    def extractMetadata(
        self, line: str, index: int, g_id: int
    ) -> dict[str, str]:
        metadata: dict[str, str] = {}
        # Extract the metadata string, e.g., " [color=red max_drones=1]"
        match = re.match(self.patterns[index], line)
        if not match:
            return metadata
        metadata_string = match.group(g_id)
        if metadata_string:
            # Remove brackets and strip whitespace
            clean_metadata_string = metadata_string.strip()[1:-1].strip()
            # Find all key=value pairs
            pairs = re.findall(r'(\w+)=(\w+)', clean_metadata_string)
            for key, value in pairs:
                metadata[key] = value
        return metadata

    def createHub(self, line: str, index: int) -> Node:
        match = re.match(self.patterns[index], line)
        if not match:
            raise Exception("Invalid hub line")
        hub: Node = Node(
            match.group(1), int(match.group(2)), int(match.group(3))
        )
        return hub

    def addMetadataToHub(self, hub: Node, metadata: dict[str, str]) -> None:
        hub.zone = metadata.get("zone", "normal")
        if hub.zone not in self.zoneTypes:
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

        start_match = re.match(self.patterns[1], line)
        if start_match:
            start: Node = self.createHub(line, 1)
            if start_match.group(4):
                start_metadata = self.extractMetadata(line, 1, 4)
                start.color = start_metadata.get("color", "")
            start.is_start = True
            start.max_drones = -1
            graph.addStart(start)
            return

        end_match = re.match(self.patterns[2], line)
        if end_match:
            end: Node = self.createHub(line, 2)
            if end_match.group(4):
                end_metadata = self.extractMetadata(line, 2, 4)
                end.color = end_metadata.get("color", "")
            end.is_end = True
            end.max_drones = -1
            graph.addEnd(end)
            return

        hub_match = re.match(self.patterns[3], line)
        if hub_match:
            hub: Node = self.createHub(line, 3)
            if hub_match.group(4):
                hub_metadata = self.extractMetadata(line, 3, 4)
                self.addMetadataToHub(hub, hub_metadata)
            graph.addNode(hub)
            return

        edge_match = re.match(self.patterns[4], line)
        if edge_match:
            edge: Edge = Edge(
                edge_match.group(1).split("-")[0],
                edge_match.group(1).split("-")[1]
            )
            if edge_match.group(2):
                edge_metadata = self.extractMetadata(line, 4, 2)
                edge.capacity = int(
                    edge_metadata.get("max_link_capacity", "1")
                )
                if edge.capacity < 1:
                    raise Exception(f"Invalid metadata in line: {line}")
            graph.addEdge(edge)
            return

    def parseFile(self, graph: Graph, filename: str) -> None:
        lines_to_parse: list[str] = []
        connection_lines: list[str] = []
        try:
            with open(filename, "r") as file:
                for line in file:
                    line = line.strip()
                    if line.startswith("#") or line == "":
                        continue
                    lines_to_parse.append(line)

            # First pass: Parse nb_drones, hubs, start_hub, and end_hub
            if not re.search(self.patterns[0], lines_to_parse[0]):
                raise Exception(
                    "file must start with nb_drones: <number_of_drones>"
                )
            self.createDrones(lines_to_parse[0])
            graph.drones = self.drones

            for line in lines_to_parse[1:]:
                if (re.search(self.patterns[1], line) or
                        re.search(self.patterns[2], line) or
                        re.search(self.patterns[3], line)):
                    # Process hubs (start_hub, end_hub, hub)
                    self.parseLine(graph, line)
                elif re.search(self.patterns[4], line):
                    # Store connection lines for the second pass
                    connection_lines.append(line)
                else:
                    raise Exception(f"Unrecognized line format: {line}")

            # Second pass: Process connection lines
            for line in connection_lines:
                self.parseLine(graph, line)

        except Exception as e:
            print(f"Error: {e}")
