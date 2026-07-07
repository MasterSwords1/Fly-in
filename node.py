from dataclasses import dataclass


@dataclass(slots=True)
class Node:
    name: str
    x: int
    y: int
    zone: str = "normal"
    cost: int = 1
    color: str = ""
    max_drones: int = 1
    is_start: bool = False
    is_end: bool = False
