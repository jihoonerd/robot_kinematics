from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3

from std_msgs.msg import Header, ColorRGBA
from viz.visualizer import FRAME_ID
import rospy

def ulink_to_marker_array(ulink) -> MarkerArray:
    marker_array = MarkerArray()
    for joints in ulink.values():
        marker = Marker(
            header=Header(frame_id=FRAME_ID, stamp=rospy.Time().now()),
            id=joints.id,
            type=Marker.SPHERE,
            action=Marker.ADD,
            pose=Pose(Point(joints.p[0], joints.p[1], joints.p[2]), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.1, 0.1, 0.1),
            color=ColorRGBA(1, 0, 0, 1),
            lifetime=rospy.Duration()
        )
        marker_array.markers.append(marker)
    return marker_array
