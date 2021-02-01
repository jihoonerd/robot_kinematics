import pickle
import os
from rk.utils import calc_vw_err

import numpy as np


class RobotObject:

    def __init__(self, ulink):
        self.ulink = ulink

    def print_link_info(self, link_id):
        
        if link_id not in self.ulink.keys():
            return KeyError("Does not have matching link node id.")
        
        querying_node = self.ulink[link_id]
        print("ID: ", querying_node.id)
        print("NAME: ", querying_node.name)
        print("Sister: ", querying_node.sister)
        print("Child: ", querying_node.child)

    def forward_kinematics(self, node_id):
        if node_id == 0: # For end of the kinematic chain. (NULL)
            return None
        if node_id != 1: # If node is not body
            mother_id = self.ulink[node_id].mother
            self.ulink[node_id].p = self.ulink[mother_id].R @ self.ulink[node_id].b + self.ulink[mother_id].p
            self.ulink[node_id].R = self.ulink[mother_id].R @ self.rodrigues(self.ulink[node_id].a, self.ulink[node_id].q)

        self.forward_kinematics(self.ulink[node_id].sister)
        self.forward_kinematics(self.ulink[node_id].child)

    def rodrigues(self, w, dt):
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
    
    def inverse_kinematics(self, to, target):
        lmbda = 0.5
        self.forward_kinematics(1)
        idx = self.find_route(to)
        for n in range(1, 11):
            J = self.calc_Jacobian(idx)
            err = calc_vw_err(target, self.ulink[to])
            if np.linalg.norm(err) < 1e-6:
                print("IK: Converged")
                return None
            dq = lmbda * np.linalg.solve(J, err)
            for nn in range(len(idx)):
                j = idx[nn]
                self.ulink[j].q = self.ulink[j].q + dq[nn]
            self.forward_kinematics(1)
        
    def find_route(self, to):
        mother_id = self.ulink[to].mother
        if mother_id == 1:
            return [to]
        else:
            return np.append(self.find_route(mother_id), [to])
    
    def set_joint_angles(self, idx, q):
        for n in range(len(idx)):
            j = idx[n]
            self.ulink[j].q = q[n]
        self.forward_kinematics(1)
    
    def calc_Jacobian(self, idx):
        """
        idx is a return of find_route (list)
        """
        jsize = len(idx)
        target = self.ulink[idx[-1]].p # absolute target position
        J = np.zeros((6, jsize))
        for i in range(jsize):
            j = idx[i]
            mom = self.ulink[j].mother
            a = self.ulink[mom].R @ self.ulink[j].a # joint axis in world frame
            J_i = np.concatenate((np.cross(a.T, (target - self.ulink[j].p).T).T, a))
            J[:, i] = J_i.squeeze()
        return J