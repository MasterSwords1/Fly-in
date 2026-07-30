from dataclasses import dataclass
from node import Node


@dataclass(slots=True)
class Edge():
    start: str
    end: str
    startNode: Node | None = None
    endNode: Node | None = None
    capacity: int = 1
