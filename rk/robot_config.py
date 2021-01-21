from dataclasses import dataclass
import numpy as np

@dataclass
class LinkNode:
    """Class for link node info"""
    name: str
    id: int
    sister: int
    child: int
    mother: int
    a: np.ndarray # joint axis vector (relative to parent)
    b: np.ndarray # joint relative position (relative to parent)
    p: np.ndarray # position in world coordinates
    q: np.ndarray # joint angle
    R: np.ndarraytwr # attitude in world coordinates

class RobotObject:

    def __init__(self):
        
        self.ulink = {
            0: LinkNode(name='OP_Middle_Hip', id=0, child=1, sister=0, mother=None),
            1: LinkNode(name='Spine', id=1, child=2, sister=10, mother=0),
            2: LinkNode(name='Thorax', id=2, child=3, sister=0, mother=1),
            3: LinkNode(name='OP_L_Shoulder', id=3, child=4, sister=6, mother=2),
            4: LinkNode(name='OP_L_Elbow', id=4, child=5, sister=0, mother=3),
            5: LinkNode(name='OP_L_Wrist', id=5, child=0, sister=0, mother=4),
            6: LinkNode(name='OP_R_Shoulder', id=6, child=7, sister=9, mother=2),
            7: LinkNode(name='OP_R_Elbow', id=7, child=8, sister=0, mother=6),
            8: LinkNode(name='OP_R_Wrist', id=8, child=0, sister=0, mother=7),
            9: LinkNode(name='OP_Neck', id=9, child=0, sister=0, mother=2),
            10: LinkNode(name='OP_L_Hip', id=10, child=11, sister=13, mother=0),
            11: LinkNode(name='OP_L_Knee', id=11, child=12, sister=0, mother=10),
            12: LinkNode(name='OP_L_Ankle', id=12, child=0, sister=0, mother=11),
            13: LinkNode(name='OP_R_Hip', id=13, child=14, sister=0, mother=0),
            14: LinkNode(name='OP_R_Knee', id=14, child=15, sister=0, mother=13),
            15: LinkNode(name='OP_R_Ankle', id=15, child=0, sister=0, mother=14),
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
       
        parent = self.ulink[node_id].mother
        self.ulink[node_id].p = self.ulink[parent.id].R * self.ulink[node_id].b + self.ulink[parent.id].p
        self.ulink[node_id].R = self.ulink[parent.id].R * self.rodrigues(self.ulink[node_id].a, self.ulink[node_id].q)

        self.forward_kinematics(self.ulink[self.ulink[node_id].sister.id])
        self.forward_kinematics(self.ulink([self.ulink[node_id].child.id])

    def rodrigues(self):
        return None
        S

if __name__ == "__main__":
    print()
    ro = RobotObject()
    for k, v in ro.ulink.items():
        print(f"{k} : {v.name} {v.id} {v.sister} {v.child} ")

    print(ro.ulink[ro.ulink[0].child].name)
    print(ro.ulink[ro.ulink[ro.ulink[1].id].sister].name)