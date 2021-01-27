from dataclasses import dataclass
import numpy as np

@dataclass
class LinkNode:
    """Class for link node info"""
    id: int
    name: str = "Untitled"
    sister: int = None
    child: int = None
    mother: int = None

    a: np.ndarray = None  # Joint axis vector (relative to parent)
    b: np.ndarray = None  # Joint relative position (relative to parent)
    p: np.ndarray = None  # Position in world coordinates
    q: np.ndarray = None  # Joint angle
    R: np.ndarray = None  # Attitude in world coordinates
    dq: np.float  = None  # Joint Velocity


def find_mother(ulink, node_id):
    if node_id != 0:
        if node_id == 1:
            ulink[node_id].mother = 0
        if ulink[node_id].child != 0:
            ulink[ulink[node_id].child].mother = node_id
            find_mother(ulink, ulink[node_id].child)
        if ulink[node_id].sister != 0:
            ulink[ulink[node_id].sister].mother = ulink[node_id].mother
            find_mother(ulink, ulink[node_id].sister)
    return ulink