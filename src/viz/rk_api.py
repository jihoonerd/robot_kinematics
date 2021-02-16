#!/usr/bin/python
import copy

import numpy as np
import rospy
from robot_kinematics.msg import IKMarker
from rk.robot_preset.biped_robot import half_sitting_biped_ro
from visualization_msgs.msg import MarkerArray
from rk.utils import LinkNode, ToRad, rpy2rot
from viz.utils import ulink_to_marker_array
from viz.viz_manager import VizManager


ik_target = dict()

def callback(data):
    global ik_target
    ik_target[data.link_id] = np.array([[data.target_pos.x, data.target_pos.y, data.target_pos.z]]).T
    print("[SUBSCRIBER MESSAGE]")
    print(ik_target)

def rk_api():
    rospy.init_node('rk_pub')
    ro = copy.deepcopy(half_sitting_biped_ro)

    pub = rospy.Publisher('rk_api/joint', MarkerArray, queue_size=10)
    sub = rospy.Subscriber('rk_api/ik_target', IKMarker, callback=callback) # use custom message

    viz_manager = VizManager()
    viz_manager.add_ik_target('7')
    viz_manager.add_ik_target('13')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        if ik_target:
            for link_id in ik_target:
                ro.inverse_kinematics(int(link_id), LinkNode(id=-1, name=link_id + '_target', p=ik_target[link_id], R=rpy2rot(0, -ToRad * 30.0, 0)))

        marker_array = ulink_to_marker_array(ro.ulink)
        pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        rk_api()
    except rospy.ROSInterruptException:
        pass
