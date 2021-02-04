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
            
            pub.publish(marker_array)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        visualizer()
    except rospy.ROSInterruptException:
        pass
