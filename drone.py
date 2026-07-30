from dataclasses import dataclass
from node import Node


@dataclass(slots=True)
class Drone():
    id: int
    position: Node | None
