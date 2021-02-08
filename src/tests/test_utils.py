import numpy as np
from numpy.testing._private.utils import assert_almost_equal
from rk.utils import LinkNode, find_mother, rpy2rot, ToRad
import pytest

@pytest.fixture
def biped_ulink():
    UX = np.array([[1, 0, 0]]).T
    UY = np.array([[0, 1, 0]]).T
    UZ = np.array([[0, 0, 1]]).T

    ro_biped_ulink = {}
    ro_biped_ulink[0] = LinkNode(id=0,  name='NULL')
    ro_biped_ulink[1] = LinkNode(id=1,  name='BODY'   , child=[2, 8], b=np.array([[0, 0, 0.7]]).T,  a=UZ, q=0)
    ro_biped_ulink[2] = LinkNode(id=2,  name='RLEG_J0', child=[3],    b=np.array([[0, -0.1, 0]]).T, a=UZ, q=0)
    ro_biped_ulink[3] = LinkNode(id=3,  name='RLEG_J1', child=[4],    b=np.array([[0, 0, 0]]).T,    a=UX, q=0)
    ro_biped_ulink[4] = LinkNode(id=4,  name='RLEG_J2', child=[5],    b=np.array([[0, 0, 0]]).T,    a=UY, q=0)
    ro_biped_ulink[5] = LinkNode(id=5,  name='RLEG_J3', child=[6],    b=np.array([[0, 0, -0.3]]).T, a=UY, q=0)
    ro_biped_ulink[6] = LinkNode(id=6,  name='RLEG_J4', child=[7],    b=np.array([[0, 0, -0.3]]).T, a=UY, q=0)
    ro_biped_ulink[7] = LinkNode(id=7,  name='RLEG_J5', child=[0],    b=np.array([[0, 0, 0]]).T,    a=UX, q=0)
    ro_biped_ulink[8] = LinkNode(id=8,  name='LLEG_J0', child=[9],    b=np.array([[0, 0.1, 0]]).T,  a=UZ, q=0)
    ro_biped_ulink[9] = LinkNode(id=9,  name='LLEG_J1', child=[10],   b=np.array([[0, 0, 0]]).T,    a=UX, q=0)
    ro_biped_ulink[10]= LinkNode(id=10, name='LLEG_J2', child=[11],   b=np.array([[0, 0, 0]]).T,    a=UY, q=0)
    ro_biped_ulink[11]= LinkNode(id=11, name='LLEG_J3', child=[12],   b=np.array([[0, 0, -0.3]]).T, a=UY, q=0)
    ro_biped_ulink[12]= LinkNode(id=12, name='LLEG_J4', child=[13],   b=np.array([[0, 0, -0.3]]).T, a=UY, q=0)
    ro_biped_ulink[13]= LinkNode(id=13, name='LLEG_J5', child=[0],    b=np.array([[0, 0, 0]]).T,    a=UX, q=0) 
    return ro_biped_ulink

def test_find_mother(biped_ulink):
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