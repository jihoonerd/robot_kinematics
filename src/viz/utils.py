from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3

from std_msgs.msg import Header, ColorRGBA
from viz.visualizer import FRAME_ID
from rk.utils import find_route
import rospy
import numpy as np

def ulink_to_marker_array(ulink) -> MarkerArray:
    marker_array = MarkerArray()
    
    # Represent joint positions by Marker.SPHERE
    marker_dict = dict()
    for joint in ulink.values():
        marker = Marker(
            header=Header(frame_id=FRAME_ID),
            id=joint.id,
            type=Marker.SPHERE,
            action=Marker.ADD,
            pose=Pose(Point(joint.p[0], joint.p[1], joint.p[2]), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.1, 0.1, 0.1),
            color=ColorRGBA(1, 0, 0, 1),
            lifetime=rospy.Duration()
        )
        marker_dict[joint.id] = marker
        marker_array.markers.append(marker)

        if joint.is_leaf:
            
            link = Marker(
                    header=Header(frame_id=FRAME_ID),
                    action=Marker.ADD,
                    id=int(str(joint.id) + '01'),
                    type=Marker.LINE_STRIP,
                    color=ColorRGBA(1.0, 1.0, 1.0, 1.0)
                )
            link.scale.x = 0.02
            link.pose.orientation.w = 1.0

            traj = np.append([1], find_route(ulink, joint.id))
            for i in traj:
                link.points.append(marker_dict[i].pose.position)
            marker_array.markers.append(link)
    return marker_array
