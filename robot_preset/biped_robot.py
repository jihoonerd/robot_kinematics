from rk.utils import LinkNode, find_mother
from rk.robot_config import RobotObject
import numpy as np

ToDeg = 180 / np.math.pi
ToRad = np.math.pi/180

UX = np.array([[1, 0, 0]]).T
UY = np.array([[0, 1, 0]]).T
UZ = np.array([[0, 0, 1]]).T

biped_ulink = {}
biped_ulink[1] = LinkNode(id=1,  name='BODY'   , sister=0, child= 2, b=np.array([[0, 0, 0.7]]).T,  a=UZ, q=0)
biped_ulink[2] = LinkNode(id=2,  name='RLEG_J0', sister=8, child= 3, b=np.array([[0, -0.1, 0]]).T, a=UZ, q=0)
biped_ulink[3] = LinkNode(id=3,  name='RLEG_J1', sister=0, child= 4, b=np.array([[0, 0, 0]]).T,    a=UX, q=0)
biped_ulink[4] = LinkNode(id=4,  name='RLEG_J2', sister=0, child= 5, b=np.array([[0, 0, 0]]).T,    a=UY, q=0)
biped_ulink[5] = LinkNode(id=5,  name='RLEG_J3', sister=0, child= 6, b=np.array([[0, 0, -0.3]]).T, a=UY, q=0)
biped_ulink[6] = LinkNode(id=6,  name='RLEG_J4', sister=0, child= 7, b=np.array([[0, 0, -0.3]]).T, a=UY, q=0)
biped_ulink[7] = LinkNode(id=7,  name='RLEG_J5', sister=0, child= 0, b=np.array([[0, 0, 0]]).T,    a=UX, q=0)
biped_ulink[8] = LinkNode(id=8,  name='LLEG_J0', sister=0, child= 9, b=np.array([[0, 0.1, 0]]).T,  a=UZ, q=0)
biped_ulink[9] = LinkNode(id=9,  name='LLEG_J1', sister=0, child=10, b=np.array([[0, 0, 0]]).T,    a=UX, q=0)
biped_ulink[10]= LinkNode(id=10, name='LLEG_J2', sister=0, child=11, b=np.array([[0, 0, 0]]).T,    a=UY, q=0)
biped_ulink[11]= LinkNode(id=11, name='LLEG_J3', sister=0, child=12, b=np.array([[0, 0, -0.3]]).T, a=UY, q=0)
biped_ulink[12]= LinkNode(id=12, name='LLEG_J4', sister=0, child=13, b=np.array([[0, 0, -0.3]]).T, a=UY, q=0)
biped_ulink[13]= LinkNode(id=13, name='LLEG_J5', sister=0, child= 0, b=np.array([[0, 0, 0]]).T,    a=UX, q=0)

# Find mother through find_mother utility
biped_ulink_wm = find_mother(biped_ulink, 1)
biped_ulink_wm[1].p = np.array([[0.0, 0.0, 0.65]]).T
biped_ulink_wm[1].R = np.eye(3)

biped_ro = RobotObject(biped_ulink_wm)
biped_ro.forward_kinematics(1)

biped_ro.ulink[1].p = np.array([[0.0, 0.0, 0.65]]).T
biped_ro.ulink[1].w = np.zeros((3,1))

for i in range(1, len(biped_ro.ulink)):
    biped_ro.ulink[i].dq = 0

# Gohalfsitting
biped_ro.ulink[4].q = -5.0 * ToRad
biped_ro.ulink[5].q = 10.0 * ToRad
biped_ro.ulink[6].q = -5.0 * ToRad

biped_ro.ulink[10].q = -5.0 * ToRad
biped_ro.ulink[11].q = 10.0 * ToRad
biped_ro.ulink[12].q = -5.0 * ToRad

biped_ro.ulink[1].p = np.array([[0.0, 0.0, 0.7]]).T
biped_ro.ulink[1].R = np.eye(3)