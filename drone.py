"""Drone module representing an individual autonomous drone in the fleet."""

from dataclasses import dataclass, field
from typing import List, Optional
from node import Node


@dataclass(slots=True)
class Drone:
    """Represents an autonomous drone in the simulation.

    Attributes:
        id: Zero-indexed integer identifier of the drone.
        position: Current Node location, or None if in transit / initialized.
        history: Sequence of location / state names visited over turns.
    """

    id: int
    position: Optional[Node] = None
    history: List[str] = field(default_factory=list)

    @property
    def label(self) -> str:
        """Returns the 1-indexed formatted string identifier (e.g. 'D1', 'D2').

        Returns:
            The standard drone identifier string.
        """
        return f"D{self.id + 1}"
