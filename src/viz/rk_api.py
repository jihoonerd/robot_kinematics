#!/usr/bin/python
import copy

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from rk.robot_preset.biped_robot import half_sitting_biped_ro
from visualization_msgs.msg import MarkerArray
from rk.utils import LinkNode, ToRad, rpy2rot
from viz.utils import ulink_to_marker_array
from viz.viz_manager import VizManager

ik_target_pos = np.array([[0, 0, 0]]).T

def callback(data):
    global ik_target_pos
    ik_target_pos = np.array([[data.x, data.y, data.z]]).T

def rk_api():
    rospy.init_node('rk_pub')
    ro = copy.deepcopy(half_sitting_biped_ro)

    pub = rospy.Publisher('rk_api/joint', MarkerArray, queue_size=10)
    sub = rospy.Subscriber('rk_api/ik_target', IKMarker, callback=callback) # use custom message

    viz_manager = VizManager()
    viz_manager.add_ik_target('LFoot')

    Lfoot = LinkNode(id=-1, name='Lfoot')
    Lfoot.p = np.array([[0.3, 0.1, 0]]).T
    Lfoot.R = rpy2rot(0, -ToRad * 30.0, 0)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        Lfoot.p = ik_target_pos
        ro.inverse_kinematics(13, Lfoot)

        marker_array = ulink_to_marker_array(ro.ulink)
        pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        rk_api()
    except rospy.ROSInterruptException:
        pass
