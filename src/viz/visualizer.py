#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker
from tf.transformations import quaternion_from_euler
import pickle

FRAME_ID = 'viz_frame'

def processFeedback(feedback):
    p = feedback.pose.position
    print(feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))

def visualizer():
    rospy.init_node('visualizer', anonymous=True)

    pub = rospy.Publisher('viz_marker', MarkerArray, queue_size=10)

    im_server = InteractiveMarkerServer('ik_marker')
    int_marker = InteractiveMarker(header=Header(frame_id='my_frame'))

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append(box_marker)
    
    # add the control to the interactive marker
    int_marker.controls.append(box_control)

    # create a control which will move the box
    # this control does not contain any markers,
    # which will cause RViz to insert two arrows
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "move_x"
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    # add the control to the interactive marker
    int_marker.controls.append(rotate_control)

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    im_server.insert(int_marker, processFeedback)
    # 'commit' changes and send to all clients
    im_server.applyChanges()

    rate = rospy.Rate(10)

    marker_array = MarkerArray()
    while not rospy.is_shutdown():

        if ulink:
            data = pickle.loads(ulink)
            joint_list = []
            for joints in data.values():
                marker = Marker(
                    header=Header(frame_id='my_frame', stamp=rospy.Time.now()),
                    id=joints.id,
                    type=Marker.SPHERE,
                    action=Marker.ADD,
                    pose=Pose(Point(joints.p[0], joints.p[1], joints.p[2]), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(0.1, 0.1, 0.1),
                    color=ColorRGBA(1, 0, 0, 1),
                    lifetime=rospy.Duration()
                )
                joint_list.append(marker)
                marker_array.markers.append(marker)
            
            # link
            line_list = Marker(
                            header=Header(frame_id='my_frame', stamp=rospy.Time.now()),
                            action=Marker.ADD,
                            id=41,
                            type=Marker.LINE_STRIP,
                            color=ColorRGBA(1.0, 1.0, 1.0, 1.0)
                        )
            line_list.scale.x = 0.02
            line_list.pose.orientation.w = 1.0
            for marker in joint_list:
                line_list.points.append(marker.pose.position)
            marker_array.markers.append(line_list)
            pub.publish(marker_array)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        visualizer()
    except rospy.ROSInterruptException:
        pass
