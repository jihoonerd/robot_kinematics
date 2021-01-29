import numpy as np
from robot_config.biped_robot import biped_ulink
from rk.robot_config import RobotObject
from robot_config.utils import find_mother


def test_find_mother():
    mother_found_ulink = find_mother(biped_ulink, 1)
    assert mother_found_ulink[1].mother == 0
    assert mother_found_ulink[2].mother == 1
    assert mother_found_ulink[3].mother == 2
    assert mother_found_ulink[4].mother == 3
    assert mother_found_ulink[5].mother == 4
    assert mother_found_ulink[6].mother == 5
    assert mother_found_ulink[7].mother == 6
    assert mother_found_ulink[8].mother == 1
    assert mother_found_ulink[9].mother == 8
    assert mother_found_ulink[10].mother == 9
    assert mother_found_ulink[11].mother == 10
    assert mother_found_ulink[12].mother == 11
    assert mother_found_ulink[13].mother == 12

def test_biped_setup():
    biped_ulink_wm = find_mother(biped_ulink, 1)
    biped_ulink_wm[1].p = np.array([[0.0, 0.0, 0.65]]).T
    biped_ulink_wm[1].R = np.eye(3)

    biped_ro = RobotObject(biped_ulink_wm)
    
    biped_ro.forward_kinematics(1)

    biped_ro.ulink[1].p = np.array([[0.0, 0.0, 0.65]]).T
    biped_ro.ulink[1].w = np.zeros((3,1))

    for i in range(1, len(biped_ro.ulink)):
        biped_ro.ulink[i].dq = 0
    
    biped_ro.visualize()