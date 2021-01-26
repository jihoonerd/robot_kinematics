from dataclasses import dataclass
import numpy as np

@dataclass
class LinkNode:
    """Class for link node info"""
    id: int
    name: str = "Untitled"
    sister: int = None
    child: int = None
    mother: int = None

    a: np.ndarray = None # joint axis vector (relative to parent)
    b: np.ndarray = None # joint relative position (relative to parent)
    p: np.ndarray = None # position in world coordinates
    q: np.ndarray = None # joint angle
    R: np.ndarray = None # attitude in world coordinates

class RobotObject:

    def __init__(self):
        
        self.ulink = {
            0: LinkNode(name="NULL", id=0), # 0 is reserved for NULL
            1: LinkNode(name='BODY', id=1, child=2, sister=0),
            2: LinkNode(name='RARM', id=2, child=3, sister=4, mother=1),
            3: LinkNode(name='RHAND', id=3, child=0, sister=0, mother=2),
            4: LinkNode(name='LARM', id=4, child=5, sister=6, mother=1),
            5: LinkNode(name='LHAND', id=5, child=0, sister=0, mother=4),
            6: LinkNode(name='RLEG', id=6, child=7, sister=8, mother=1),
            7: LinkNode(name='RFOOT', id=7, child=0, sister=0, mother=6),
            8: LinkNode(name='LLEG', id=8, child=9, sister=0, mother=1),
            9: LinkNode(name='LFOOT', id=9, child=0, sister=0, mother=8),
        }

    def print_link_name(self, link_id):
        
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
            mother = self.ulink[node_id].mother
            self.ulink[node_id].p = self.ulink[mother.id].R @ self.ulink[node_id].b + self.ulink[mother.id].p
            self.ulink[node_id].R = self.ulink[mother.id].R @ self.rodrigues(self.ulink[node_id].a, self.ulink[node_id].q)

        self.forward_kinematics(self.ulink[node_id].sister.id)
        self.forward_kinematics(self.ulink[node_id].child.id)

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
            th = norm_w @ dt # amount of rotation (rad)
            w_wedge = np.array([[0, -wn[3], wn[2]], [wn[3], 0, -wn[1]], [-wn[2], wn[1], 0]])
            R = np.eye(3) + w_wedge * np.sin(th) + np.linalg.matrix_power(w_wedge, 2) * (1-np.cos(th))
        return R

if __name__ == "__main__":
    print()
    ro = RobotObject()
    for k, v in ro.ulink.items():
        print(f"{k} : {v.name} {v.id} {v.sister} {v.child} ")

    print(ro.ulink[ro.ulink[2].child].name)
    print(ro.ulink[ro.ulink[ro.ulink[2].id].sister].name)