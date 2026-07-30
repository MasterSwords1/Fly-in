from node import Node
from edge import Edge
from drone import Drone


class Graph():
    def __init__(self) -> None:
        self.nodes: dict[str, Node] = {}
        self.edges: list[Edge] = []
        self.drones: list[Drone] = []
        self.startNode: Node | None = None
        self.endNode: Node | None = None
        self.connections: set[tuple[str, str]] = set()

    def addNode(self, node: Node) -> None:
        if self.nodes.get(node.name, None):
            raise Exception(f"duplicate hub name: {node.name}")
        self.nodes[node.name] = node

    def addStart(self, start: Node) -> None:
        if self.startNode:
            raise Exception(f"Start Hub redifinition: {start.name}")
        self.startNode = start
        self.nodes[start.name] = start

    def addEnd(self, end: Node) -> None:
        if self.endNode:
            raise Exception(f"End Hub redifinition: {end.name}")
        self.endNode = end
        self.nodes[end.name] = end

    def addEdge(self, edge: Edge) -> None:
        if not self.nodes.get(edge.start, None):
            raise Exception(
                f"Connection start hub name does not exist: {edge.start}"
            )
        if not self.nodes.get(edge.end, None):
            raise Exception(
                f"Connection end hub name does not exist: {edge.end}"
            )

        # Canonicalize the connection names to handle a-b and b-a as duplicates
        canonical_connection: tuple[str, str] = (
            min(edge.start, edge.end), max(edge.start, edge.end)
        )
        if canonical_connection in self.connections:
            raise Exception(
                f"Duplicate connection found: {edge.start}-{edge.end}"
            )

        self.connections.add(canonical_connection)
        edge.startNode = self.nodes.get(edge.start, None)
        edge.endNode = self.nodes.get(edge.end, None)
        self.edges.append(edge)
