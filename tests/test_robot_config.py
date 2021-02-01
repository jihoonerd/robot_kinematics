import copy

import numpy as np
from rk.utils import LinkNode, find_mother, rpy2rot, ToRad
from robot_preset.biped_robot import biped_ro, half_sitting_biped_ro


def test_find_mother():
    mother_found_ulink = find_mother(biped_ro.ulink, 1)
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

def test_jacobian_setting():
    ro = copy.deepcopy(biped_ro)
    idx = ro.find_route(7)
    ro.set_joint_angles(idx, [0, 0, -np.math.pi/6, np.math.pi/3, -np.math.pi/6, 0])
    J = ro.calc_Jacobian(idx)
    dq = np.linalg.solve(J, np.array([0, 0, 0.1, 0, 0, 0]).T)
    np.testing.assert_almost_equal(dq , np.array([0, 0, -1/3, 2/3, -1/3, 0]))

def test_singular_postures():
    ro = copy.deepcopy(biped_ro)
    idx = ro.find_route(7)
    ro.set_joint_angles(idx, [0, 0, -np.math.pi/6, np.math.pi/3, np.math.pi/3, 0])
    J = ro.calc_Jacobian(idx)
    np.testing.assert_almost_equal(np.linalg.det(J), 8.907937896779329e-18)
    np.testing.assert_equal(np.linalg.matrix_rank(J), 5)

def test_inverse_kinematics_half_sitting():
    ro = copy.deepcopy(half_sitting_biped_ro)
    
    Rfoot = LinkNode(id=-1, name='Rfoot')
    Rfoot.p = np.array([[-0.3, -0.1, 0]]).T
    Rfoot.R = rpy2rot(0, ToRad * 20.0, 0)
    ro.inverse_kinematics(7, Rfoot)

    Lfoot = LinkNode(id=-1, name='Lfoot')
    Lfoot.p = np.array([[0.3, 0.1, 0]]).T
    Lfoot.R = rpy2rot(0, -ToRad * 30.0, 0)
    ro.inverse_kinematics(13, Lfoot)

def test_inverse_kinematics_LM_half_sitting():
    ro = copy.deepcopy(half_sitting_biped_ro)
    
    Rfoot = LinkNode(id=-1, name='Rfoot')
    Rfoot.p = np.array([[-0.3, -0.1, 0]]).T
    Rfoot.R = rpy2rot(0, ToRad * 20.0, 0)
    ro.inverse_kinematics_LM(7, Rfoot)

    Lfoot = LinkNode(id=-1, name='Lfoot')
    Lfoot.p = np.array([[0.3, 0.1, 0]]).T
    Lfoot.R = rpy2rot(0, -ToRad * 30.0, 0)
    ro.inverse_kinematics_LM(13, Lfoot)
