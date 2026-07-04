from dataclasses import dataclass
from node import Node
@dataclass(slots=True)
class Edge():
    start: Node
    end: Node
    capacity: int = 1