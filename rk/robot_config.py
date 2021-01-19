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
            0: LinkNode(name='body', id=0, sister=0, child=2),
            2: LinkNode(name='rarm', id=2, sister=4, child=3),
            3: LinkNode(name='rhand', id=3, sister=0, child=0),
            4: LinkNode(name='larm', id=4, sister=6, child=5),
            5: LinkNode(name='lhand', id=5, sister=0, child=0),
            6: LinkNode(name='rleg', id=6, sister=8, child=7),
            7: LinkNode(name='rfoot', id=7, sister=0, child=0),
            8: LinkNode(name='lleg', id=8, sister=0, child=9),
            9: LinkNode(name='lfoot', id=9, sister=0, child=0)
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