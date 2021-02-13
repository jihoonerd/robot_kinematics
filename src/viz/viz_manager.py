import rospy
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker
from viz.visualizer import FRAME_ID
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3


class VizManager:

    def __init__(self):
        self.im_server = None
        self.ik_target_pub = None

        self.init_im_server()
        self.init_ik_target_pub()
        self.im = dict()

    def init_im_server(self):
        self.im_server = InteractiveMarkerServer('im_server')
    
    def init_ik_target_pub(self):
        self.ik_target_pub = rospy.Publisher('rk_api/ik_target', Vector3, queue_size=10)

    def add_im(self, name: str):
        self.im[name] = InteractiveMarker(header=Header(frame_id=FRAME_ID), name=name, scale=0.25)

        box_marker = Marker(
                    header=Header(frame_id=FRAME_ID, stamp=rospy.Time.now()),
                    type=Marker.CUBE,
                    action=Marker.ADD,
                    pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(0.1, 0.1, 0.1),
                    color=ColorRGBA(1, 1, 0, 0.5),
                    lifetime=rospy.Duration()
                )
    
        box_control = InteractiveMarkerControl(always_visible=True)
        box_control.markers.append(box_marker)
        self.im[name].controls.append(box_control)

        move_x_control = InteractiveMarkerControl(
            orientation=Quaternion(1, 0, 0, 1)
        )
        move_x_control.name = 'move_x'
        move_x_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.im[name].controls.append(move_x_control)

        move_y_control = InteractiveMarkerControl(
            orientation=Quaternion(0, 0, 1, 1)
        )
        move_y_control.name = 'move_y'
        move_y_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.im[name].controls.append(move_y_control)

        move_z_control = InteractiveMarkerControl(
            orientation=Quaternion(0, 1, 0, 1)
        )
        move_z_control.name = 'move_z'
        move_z_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.im[name].controls.append(move_z_control)

        self.im_server.insert(self.im[name], self.process_feedback)
        self.im_server.applyChanges()
    
    def process_feedback(self, feedback):

        x = feedback.pose.position.x
        y = feedback.pose.position.y
        z = feedback.pose.position.z

        target_pos = Vector3(x, y, z)
        self.ik_target_pub.publish(target_pos)

        s = "Marker Name: " + feedback.marker_name
        s += " / control: " + feedback.control_name
        s += " / position " + f"{x}, {y}, {z}"
        print(s)
        

    
        