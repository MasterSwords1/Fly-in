from node import Node
from edge import Edge

class Graph():
    def __init__(self) -> None:
        self.nodes: dict[str, Node] = {}
        self.edges: list[Edge] = []
        self.startNode: Node | None = None
        self.endNode: Node | None = None
    
    def addNode(self, node: Node) -> None:
        if not self.nodes[node.name]:
            self.nodes[node.name] = node
        raise Exception(f"duplicate hub name: {node.name}")
    
    def addStart(self, start: Node) -> None:
        if self.startNode:
            raise Exception("Start Hub redifinition")
        self.startNode = start
    
    def addEnd(self, end: Node) -> None:
        if self.startNode:
            raise Exception("End Hub redifinition")
        self.startNode = end

    def addEdge(self, edge: Edge) -> None:
        if not self.nodes[edge.start.name]:
            raise Exception("Connection start hub name does not exist")
        if not self.nodes[edge.end.name]:
            raise Exception("Connection end hub name does not exist")
        self.edges.append(edge)
