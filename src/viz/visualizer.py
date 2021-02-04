#!/usr/bin/python
import rospy
import os
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import pathlib
import pickle
import socket
import sys


VIZ_PATH = pathlib.Path(__file__).parent.absolute()
ULINK_FN = 'ulink.pkl'

HOST = '127.0.0.1'
PORT = 3000

if not os.path.exists(VIZ_PATH):
    pathlib.Path(VIZ_PATH).mkdir(parents=True, exist_ok=True)

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
            
            pub.publish(marker_array)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        visualizer()
    except rospy.ROSInterruptException:
        pass
