from node import Node
from edge import Edge

class Graph():
    def __init__(self) -> None:
        self.nodes: dict[str, Node] = {}
        self.edges: list[Edge] = []
    def addNode(self, node: Node) -> None:
        if not self.nodes[node.name]:
            self.nodes[node.name] = node
        raise Exception(f"duplicate hub name: {node.name}")