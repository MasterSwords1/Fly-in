"""Edge module representing bidirectional connections between zones."""

from dataclasses import dataclass
from typing import Optional
from node import Node


@dataclass(slots=True)
class Edge:
    """Represents a bidirectional connection between two zones.

    Attributes:
        start: Name of the first connected zone.
        end: Name of the second connected zone.
        start_node: Direct reference to the start Node object, if resolved.
        end_node: Direct reference to the end Node object, if resolved.
        capacity: Maximum number of drones traversing simultaneously.
    """

    start: str
    end: str
    start_node: Optional[Node] = None
    end_node: Optional[Node] = None
    capacity: int = 1

    @property
    def name(self) -> str:
        """Returns the canonical connection name string.

        Returns:
            The formatted connection name as '<start>-<end>'.
        """
        return f"{self.start}-{self.end}"

    def canonical_pair(self) -> tuple[str, str]:
        """Returns a sorted tuple of endpoint names to identify uniqueness.

        Returns:
            Alphabetically ordered tuple of (name1, name2).
        """
        return (min(self.start, self.end), max(self.start, self.end))

    def connects(self, node_name: str) -> bool:
        """Checks if this edge connects to the given zone name.

        Args:
            node_name: Name of the zone to check.

        Returns:
            True if node_name is either endpoint of this edge.
        """
        return self.start == node_name or self.end == node_name

    def get_other_endpoint(self, node_name: str) -> Optional[str]:
        """Returns the opposite endpoint from the given zone name.

        Args:
            node_name: Name of one endpoint.

        Returns:
            The opposite endpoint zone name, or None if node_name is not on this edge.
        """
        if self.start == node_name:
            return self.end
        if self.end == node_name:
            return self.start
        return None
