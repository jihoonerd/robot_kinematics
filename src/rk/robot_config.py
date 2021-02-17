import numpy as np
from rk.utils import calc_vw_err, rodrigues


class RobotObject:

    def __init__(self, ulink):
        self.ulink = ulink
    
    def print_link_info(self, link_id):
        
        if link_id not in self.ulink.keys():
            return KeyError("Does not have matching link node id.")
        
        querying_node = self.ulink[link_id]
        print("ID: ", querying_node.id)
        print("NAME: ", querying_node.name)
        print("Children: ", querying_node.children)

    def forward_kinematics(self, node_id):
        if node_id == 0: # For end of the kinematic chain. (NULL)
            return None
        if node_id != 1: # If node is not body
            mother_id = self.ulink[node_id].mother
            self.ulink[node_id].p = (self.ulink[mother_id].R @ self.ulink[node_id].b + self.ulink[mother_id].p).astype(float)
            self.ulink[node_id].R = (self.ulink[mother_id].R @ rodrigues(self.ulink[node_id].a, self.ulink[node_id].q)).astype(float)

        for child_id in self.ulink[node_id].children:
            self.forward_kinematics(child_id)
            
    def inverse_kinematics_LM(self, to, target):
        """Levenberg-Marquardt, Chan-Lawrence, Sugihara's modification"""
        idx = self.find_route(to)
        wn_pos = 1 / 0.3
        wn_ang = 1 / (2 * np.math.pi)
        We = np.diag([wn_pos, wn_pos, wn_pos, wn_ang, wn_ang, wn_ang])
        Wn = np.eye(len(idx))

        self.forward_kinematics(1)
        err = calc_vw_err(target, self.ulink[to])
        Ek = err.T @ We @ err

        for _ in range(10):
            J = self.calc_Jacobian(idx)
            lmbda = Ek + 0.002
            Jh = J.T @ We @ J + Wn * lmbda
            
            gerr = J.T @ We @ err # gk
            dq = np.linalg.solve(Jh, gerr) # new

            self.move_joints(idx, dq)
            self.forward_kinematics(1)
            err = calc_vw_err(target, self.ulink[to])
            Ek2 = err.T @ We @ err

            if Ek2 < 1e-12:
                break
            elif Ek2 < Ek:
                Ek = Ek2
            else:
                self.move_joints(idx, -dq) # revert
                self.forward_kinematics(1)
                break
        err_norm = np.linalg.norm(err)
        return err_norm

    def inverse_kinematics(self, to, target):
        lmbda = 0.9
        idx = self.find_route(to)
        self.forward_kinematics(1)
        err = calc_vw_err(target, self.ulink[to])

        for n in range(10):
            if np.linalg.norm(err) < 1e-6:
                break
            J = self.calc_Jacobian(idx)
            dq = lmbda * np.linalg.solve(J, err)
            self.move_joints(idx, dq)
            self.forward_kinematics(1)
            err = calc_vw_err(target, self.ulink[to])
        err_norm = np.linalg.norm(err)
        return err_norm
    
    def move_joints(self, idx, dq):
        for i in range(len(idx)):
            j = idx[i]
            self.ulink[j].q = self.ulink[j].q + dq[i]

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
