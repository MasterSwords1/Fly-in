from dataclasses import dataclass
from node import Node

@dataclass(slots=True)
class Edge():
    start: str
    end: str
    capacity: int = 1