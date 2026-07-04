from dataclasses import dataclass
from enum import Enum

class ZoneType(Enum):
    NORMAL = "normal"
    RESTRICTED = "restricted"
    BLOCKED = "blocked"
    PRIORITY = "priority"

@dataclass(slots=True)
class Node:
    name: str
    x: int
    y: int
    zone: ZoneType = ZoneType.NORMAL | None
    cost: int = 1
    color: str = ""
    max_drone: int = 1
    is_start: bool = False
    is_end: bool = False
