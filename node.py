"""Node module representing network hubs and zones."""

from dataclasses import dataclass
from typing import Final

# Zone cost constants
DEFAULT_NORMAL_COST: Final[int] = 1
DEFAULT_RESTRICTED_COST: Final[int] = 2
DEFAULT_PRIORITY_COST: Final[int] = 1


@dataclass(slots=True)
class Node:
    """Represents a hub/zone in the drone network.

    Attributes:
        name: Unique name identifier of the zone.
        x: Integer X coordinate on the map grid.
        y: Integer Y coordinate on the map grid.
        zone: Zone type ('normal', 'blocked', 'restricted', 'priority').
        cost: Turn cost to traverse into this zone.
        color: Visual display color name or hex code.
        max_drones: Maximum simultaneous drone capacity.
        is_start: Flag indicating if this is the start hub.
        is_end: Flag indicating if this is the end/goal hub.
    """

    name: str
    x: int
    y: int
    zone: str = "normal"
    cost: int = DEFAULT_NORMAL_COST
    color: str = ""
    max_drones: int = 1
    is_start: bool = False
    is_end: bool = False

    def is_blocked(self) -> bool:
        """Checks if the zone is blocked and inaccessible.

        Returns:
            True if the zone is blocked, False otherwise.
        """
        return self.zone == "blocked"

    def is_restricted(self) -> bool:
        """Checks if the zone requires 2 turns to enter.

        Returns:
            True if the zone is restricted, False otherwise.
        """
        return self.zone == "restricted"

    def is_priority(self) -> bool:
        """Checks if the zone has priority status in pathfinding.

        Returns:
            True if the zone is a priority zone, False otherwise.
        """
        return self.zone == "priority"
