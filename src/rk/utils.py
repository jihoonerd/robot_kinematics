from dataclasses import dataclass
import numpy as np

ToDeg = 180 / np.math.pi
ToRad = np.math.pi/180

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

def rpy2rot(roll, pitch, yaw):
    """Implementation of (2.13)"""
    Cphi = np.math.cos(roll)
    Sphi = np.math.sin(roll)
    Cthe = np.math.cos(pitch)
    Sthe = np.math.sin(pitch)
    Cpsi = np.math.cos(yaw)
    Spsi = np.math.sin(yaw)

    rot = np.array([
        [Cpsi * Cthe, -Spsi * Cphi + Cpsi * Sthe * Sphi, Spsi * Sphi + Cpsi * Sthe * Cphi],
        [Spsi * Cthe, Cpsi * Cphi + Spsi * Sthe * Sphi, -Cpsi * Sphi + Spsi * Sthe * Cphi],
        [-Sthe, Cthe * Sphi, Cthe * Cphi]
    ])
    assert rot.shape == (3, 3)
    return rot

def rot2omega(R):
    el = np.array([
            [R[2,1] - R[1,2]],
            [R[0,2] - R[2,0]], 
            [R[1,0] - R[0,1]]
        ])
    norm_el = np.linalg.norm(el)
    if norm_el > 1e-10:
        w = np.arctan2(norm_el, np.trace(R)-1) / norm_el * el
    elif R[0,0] > 0 and R[1,1] > 0 and R[2,2] > 0:
        w = np.array([[0, 0, 0]]).T
    else:
        w = np.math.pi/2 * np.array([[R[0,0]+1], [R[1,1]+1], [R[2,2]+1]])
    return w

def calc_vw_err(cref, cnow):
    
    perr = cref.p - cnow.p
    Rerr = np.linalg.inv(cnow.R) @ cref.R
    werr = cnow.R @ rot2omega(Rerr)
    return np.concatenate([perr, werr], axis=0)