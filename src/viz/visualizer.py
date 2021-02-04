#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
import pickle
import socket


HOST = '127.0.0.1'
PORT = 3000

def visualizer():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)
    client_socket, addr = server_socket.accept()
    print('Connected by', addr)

    pub = rospy.Publisher('viz_marker', MarkerArray, queue_size=10)
    rospy.init_node('viz_node', anonymous=True)
    rate = rospy.Rate(10)

    marker_array = MarkerArray()
    while not rospy.is_shutdown():
        ulink = client_socket.recv(1024 * 8)

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

                # Add gizmos

                x_gizmo = Marker(
                    header=Header(frame_id='my_frame', stamp=rospy.Time.now()),
                    id=91,
                    type=Marker.ARROW,
                    action=Marker.ADD,
                    pose=Pose(Point(-2, -2, 0), Quaternion(*quaternion_from_euler(0, 0, 0))),
                    scale=Vector3(1, 0.1, 0.05),
                    color=ColorRGBA(1, 0, 0, 1),
                    lifetime=rospy.Duration()                    
                )
                y_gizmo = Marker(
                    header=Header(frame_id='my_frame', stamp=rospy.Time.now()),
                    id=92,
                    type=Marker.ARROW,
                    action=Marker.ADD,
                    pose=Pose(Point(-2, -2, 0), Quaternion(*quaternion_from_euler(0, 0, np.math.pi/2))),
                    scale=Vector3(1, 0.1, 0.05),
                    color=ColorRGBA(0, 1, 0, 1),
                    lifetime=rospy.Duration()                    
                )
                z_gizmo = Marker(
                    header=Header(frame_id='my_frame', stamp=rospy.Time.now()),
                    id=93,
                    type=Marker.ARROW,
                    action=Marker.ADD,
                    pose=Pose(Point(-2, -2, 0), Quaternion(*quaternion_from_euler(0, -np.math.pi/2, 0))),
                    scale=Vector3(1, 0.1, 0.05),
                    color=ColorRGBA(0, 0, 1, 1),
                    lifetime=rospy.Duration()                    
                )

                marker_array.markers.append(x_gizmo)
                marker_array.markers.append(y_gizmo)
                marker_array.markers.append(z_gizmo)
            
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
