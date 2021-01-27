from robot_config.utils import LinkNode, find_mother
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
