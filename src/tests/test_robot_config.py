import copy

import numpy as np
from rk.robot_preset.biped_robot import biped_ro, half_sitting_biped_ro
from rk.utils import LinkNode, ToRad, rpy2rot, find_route


def test_jacobian_setting():
    ro = copy.deepcopy(biped_ro)
    idx = find_route(ro.ulink, 7)
    ro.set_joint_angles(idx, [0, 0, -np.math.pi/6, np.math.pi/3, -np.math.pi/6, 0])
    J = ro.calc_Jacobian(idx)
    dq = np.linalg.solve(J, np.array([0, 0, 0.1, 0, 0, 0]).T)
    np.testing.assert_almost_equal(dq , np.array([0, 0, -1/3, 2/3, -1/3, 0]))

def test_singular_postures():
    ro = copy.deepcopy(biped_ro)
    idx = find_route(ro.ulink, 7)
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

    np.testing.assert_almost_equal(
        ro.ulink[7].p, 
        np.array([[-0.22863816], [-0.1], [ 0.16902202]])
    )

    np.testing.assert_almost_equal(
        ro.ulink[13].p,
        np.array([[0.22863817], [0.1], [0.169022]])
    )
    

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

    np.testing.assert_almost_equal(
        ro.ulink[7].p, 
        np.array([[-0.23635643], [-0.1], [0.14851507]])
    )

    np.testing.assert_almost_equal(
        ro.ulink[13].p,
        np.array([[0.23635229], [0.1], [0.14851329]])
    )
