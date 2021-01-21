from dataclasses import dataclass

@dataclass
class LinkNode:
    """Class for link node info"""
    name: str
    id: int
    sister: int
    child: int


class RobotObject:

    def __init__(self):
        self.ulink = {
            0: LinkNode(name='OP_Middle_Hip', id=0, child=1, sister=0),
            1: LinkNode(name='Spine', id=1, child=2, sister=10),
            2: LinkNode(name='Thorax', id=2, child=3, sister=0),
            3: LinkNode(name='OP_L_Shoulder', id=3, child=4, sister=6),
            4: LinkNode(name='OP_L_Elbow', id=4, child=5, sister=0),
            5: LinkNode(name='OP_L_Wrist', id=5, child=0, sister=0),
            6: LinkNode(name='OP_R_Shoulder', id=6, child=7, sister=9),
            7: LinkNode(name='OP_R_Elbow', id=7, child=8, sister=0),
            8: LinkNode(name='OP_R_Wrist', id=8, child=0, sister=0),
            9: LinkNode(name='OP_Neck', id=9, child=0, sister=0),
            10: LinkNode(name='OP_L_Hip', id=10, child=11, sister=13),
            11: LinkNode(name='OP_L_Knee', id=11, child=12, sister=0),
            12: LinkNode(name='OP_L_Ankle', id=12, child=0, sister=0),
            13: LinkNode(name='OP_R_Hip', id=13, child=14, sister=0),
            14: LinkNode(name='OP_R_Knee', id=14, child=15, sister=0),
            15: LinkNode(name='OP_R_Ankle', id=15, child=0, sister=0),
        }

    def print_link_name(self, link_id):
        
        if link_id not in self.ulink.keys():
            return KeyError("Does not have matching link node id.")
        
        querying_node = self.ulink[link_id]
        print("ID: ", querying_node.id)
        print("NAME: ", querying_node.name)
        print("Sister: ", querying_node.sister)
        print("Child: ", querying_node.child)

if __name__ == "__main__":
    print()
    ro = RobotObject()
    for k, v in ro.ulink.items():
        print(f"{k} : {v.name} {v.id} {v.sister} {v.child} ")

    print(ro.ulink[ro.ulink[0].child].name)