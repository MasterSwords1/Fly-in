"""Graph module managing the zone network topology, capacities, and drone fleet."""

from typing import Dict, List, Optional, Set, Tuple
from node import Node
from edge import Edge
from drone import Drone


class GraphError(Exception):
    """Exception raised for graph topology and validation errors."""


class Graph:
    """Represents the complete network graph of zones, connections, and drones."""

    def __init__(self) -> None:
        """Initializes an empty Graph instance."""
        self.nodes: Dict[str, Node] = {}
        self.edges: List[Edge] = []
        self.drones: List[Drone] = []
        self.start_node: Optional[Node] = None
        self.end_node: Optional[Node] = None
        self.connections: Set[Tuple[str, str]] = set()
        self._adjacency: Dict[str, List[str]] = {}
        self._edge_map: Dict[Tuple[str, str], Edge] = {}

    @property
    def startNode(self) -> Optional[Node]:
        """Backward-compatible alias for start_node."""
        return self.start_node

    @startNode.setter
    def startNode(self, node: Optional[Node]) -> None:
        self.start_node = node

    @property
    def endNode(self) -> Optional[Node]:
        """Backward-compatible alias for end_node."""
        return self.end_node

    @endNode.setter
    def endNode(self, node: Optional[Node]) -> None:
        self.end_node = node

    def add_node(self, node: Node) -> None:
        """Adds a regular zone node to the network.

        Args:
            node: Node object to register.

        Raises:
            GraphError: If a node with the same name already exists.
        """
        if node.name in self.nodes:
            raise GraphError(f"Duplicate hub name: {node.name}")
        self.nodes[node.name] = node
        self._adjacency.setdefault(node.name, [])

    def add_start(self, start: Node) -> None:
        """Sets the unique starting zone for all drones.

        Args:
            start: Node object representing the start hub.

        Raises:
            GraphError: If a start hub has already been defined.
        """
        if self.start_node is not None:
            raise GraphError(f"Start hub redefinition: {start.name}")
        start.is_start = True
        self.start_node = start
        self.add_node(start)

    def add_end(self, end: Node) -> None:
        """Sets the unique end/target zone for all drones.

        Args:
            end: Node object representing the goal hub.

        Raises:
            GraphError: If an end hub has already been defined.
        """
        if self.end_node is not None:
            raise GraphError(f"End hub redefinition: {end.name}")
        end.is_end = True
        self.end_node = end
        self.add_node(end)

    def add_edge(self, edge: Edge) -> None:
        """Adds a bidirectional connection between two defined zones.

        Args:
            edge: Edge object to add.

        Raises:
            GraphError: If endpoints do not exist or connection is a duplicate.
        """
        if edge.start not in self.nodes:
            raise GraphError(f"Connection start hub does not exist: {edge.start}")
        if edge.end not in self.nodes:
            raise GraphError(f"Connection end hub does not exist: {edge.end}")

        pair = edge.canonical_pair()
        if pair in self.connections:
            raise GraphError(f"Duplicate connection found: {edge.start}-{edge.end}")

        self.connections.add(pair)
        edge.start_node = self.nodes[edge.start]
        edge.end_node = self.nodes[edge.end]
        self.edges.append(edge)

        self._adjacency.setdefault(edge.start, []).append(edge.end)
        self._adjacency.setdefault(edge.end, []).append(edge.start)
        self._edge_map[(edge.start, edge.end)] = edge
        self._edge_map[(edge.end, edge.start)] = edge

    def addNode(self, node: Node) -> None:
        """Backward-compatible camelCase method."""
        self.add_node(node)

    def addStart(self, start: Node) -> None:
        """Backward-compatible camelCase method."""
        self.add_start(start)

    def addEnd(self, end: Node) -> None:
        """Backward-compatible camelCase method."""
        self.add_end(end)

    def addEdge(self, edge: Edge) -> None:
        """Backward-compatible camelCase method."""
        self.add_edge(edge)

    def get_neighbors(self, node_name: str) -> List[str]:
        """Returns the list of adjacent zone names for a given node.

        Args:
            node_name: Name of the zone.

        Returns:
            List of neighbor zone names.
        """
        return self._adjacency.get(node_name, [])

    def get_edge(self, u: str, v: str) -> Optional[Edge]:
        """Retrieves the edge connecting zones u and v if it exists.

        Args:
            u: First zone name.
            v: Second zone name.

        Returns:
            The connecting Edge object or None.
        """
        return self._edge_map.get((u, v))

    def validate(self) -> None:
        """Validates network completeness (start and end nodes present).

        Raises:
            GraphError: If start or end node is missing.
        """
        if self.start_node is None:
            raise GraphError("Missing start_hub in graph definition")
        if self.end_node is None:
            raise GraphError("Missing end_hub in graph definition")
