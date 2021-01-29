import pickle
import os

import numpy as np
from visualize.canvas import Canvas


class RobotObject:

    def __init__(self, ulink):
        self.ulink = ulink
        self.canvas = Canvas()

    def print_link_name(self, link_id):
        
        if link_id not in self.ulink.keys():
            return KeyError("Does not have matching link node id.")
        
        querying_node = self.ulink[link_id]
        print("ID: ", querying_node.id)
        print("NAME: ", querying_node.name)
        print("Sister: ", querying_node.sister)
        print("Child: ", querying_node.child)

    def write_ulink(self):
        if not os.path.exists('ulink_pkl'):
            os.mkdir('ulink_pkl')
        
        with open(os.path.join('ulink_pkl', 'ulink.pkl'), 'wb') as fp:
            pickle.dump(self.ulink, fp)

    def forward_kinematics(self, node_id):
        if node_id == 0: # For end of the kinematic chain. (NULL)
            return None
        if node_id != 1: # If node is not body
            mother_id = self.ulink[node_id].mother
            self.ulink[node_id].p = self.ulink[mother_id].R @ self.ulink[node_id].b + self.ulink[mother_id].p
            self.ulink[node_id].R = self.ulink[mother_id].R @ self.rodrigues(self.ulink[node_id].a, self.ulink[node_id].q)

        self.forward_kinematics(self.ulink[node_id].sister)
        self.forward_kinematics(self.ulink[node_id].child)
        self.write_ulink()

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
            J = self.calc_jacobian(idx)
            err = self.calc_vw_err(target, self.ulink[to])
            if np.linalg.norm(err) < 1e-6:
                print("IK: Converged")
                return None
            dq = lmbda * np.linalg.lstsq(J, err)
            for nn in range(1, len(idx) + 1): # TODO: make sure indexing
                j = idx[nn]
                self.ulink[j].q = self.ulink[j].q + dq[nn]
            self.forward_kinematics(1)
        
    def find_route(self, to):
        mother_id = self.ulink[to].mother
        if to == 1:
            return [to]
        else:
            return np.append([to], self.find_route(mother_id))
    
    def set_joint_angles(self, idx, q):
        for n in range(1, len(idx) +1): # TODO: make sure indexing
            j = idx[n]
            self.ulink[j].q = q[n]
        self.forward_kinematics(1)

    def calc_vw_err(self, cref, cnow):
        
        perr = cref.p- cnow.p
        Rerr = np.linalg.inv(cnow.R) @ cref.R
        werr = cnow.R @ self.rot2omega(Rerr)
        return [perr, werr]
    
    def rot2omega(self, R):
        el = np.array([
                [R[2,1] - R[1,2]],
                [R[0,2] - R[2,0]], 
                [R[1,0] - R[0,1]]
            ])
        norm_el = np.linalg.norm(el)
        if norm_el > 1e-10:
            w = np.arctan2(norm_el, np.trace(R)-1) / norm_el @ el
        elif R[0,0] > 0 & R(1,1) > 0 & R(2,2) > 0:
            w = np.array([[0, 0, 0]]).T
        else:
            w = np.math.pi/2 * np.array([[R[0,0]+1], [R[1,1]+1], [R[2,2]+1]])
        return w

    def visualize(self):
        self.canvas.animate()
