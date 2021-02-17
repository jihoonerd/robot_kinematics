from dataclasses import dataclass
import numpy as np
from typing import List


@dataclass
class LinkNode:
    """Class for link node info"""
    id: int
    name: str = "Untitled"
    children: List = None
    mother: int = None

    is_leaf: bool = False

    a: np.ndarray = None  # Joint axis vector (relative to parent)
    b: np.ndarray = None  # Joint relative position (relative to parent)
    p: np.ndarray = None  # Position in world coordinates
    q: np.ndarray = None  # Joint angle
    R: np.ndarray = None  # Attitude in world coordinates
    dq: np.float  = None  # Joint Velocity
    v: np.ndarray = None  # Linear Velocity in World Coordinate
    w: np.ndarray = None  # Angular Velocity in World Coordinates


def find_mother(ulink, node_id):
    """It recusrively map mother of kinematic chain. Mostly root id will be given as the node_id"""
    if node_id != 0:  # 0 node_id means NULL
        if node_id == 1:
            ulink[node_id].mother = 0
        if ulink[node_id].children:  # if children list is not empty
            for child_id in ulink[node_id].children:
                if child_id == 0:
                    continue
                ulink[child_id].mother = node_id
                find_mother(ulink, child_id)
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

def rodrigues(w, dt):
    """This returns SO(3) from so(3)

    Args:
        w: should be a (3,1) size vector
        dt ([type]): [description]

    Returns:
        [type]: [description]
    """
    norm_w = np.linalg.norm(w)
    if norm_w < 1e-10: # TODO: Find more consistent way to manage epsilon
        R = np.eye(3)
    else:
        wn = w/norm_w # rotation axis (unit vector)
        th = norm_w * dt # amount of rotation (rad)
        w_wedge = np.array([[0, -wn[2], wn[1]], [wn[2], 0, -wn[0]], [-wn[1], wn[0], 0]])
        R = np.eye(3) + w_wedge * np.sin(th) + np.linalg.matrix_power(w_wedge, 2) * (1-np.cos(th))
    return R

def find_route(ulink, query):
    mother_id = ulink[query].mother
    if mother_id == 1:
        return [query]
    else:
        return np.append(find_route(ulink, mother_id), [query])