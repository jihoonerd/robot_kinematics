#!/usr/bin/python
import rospy
import copy
from visualization_msgs.msg import MarkerArray
from viz.utils import ulink_to_marker_array
from viz.viz_manager import VizManager
from rk.robot_preset.biped_robot import half_sitting_biped_ro

def rk_api():
    rospy.init_node('rk_pub')
    ro = copy.deepcopy(half_sitting_biped_ro)

    pub = rospy.Publisher('rk_api/joint', MarkerArray, queue_size=10)
    viz_manager = VizManager()
    viz_manager.add_im('LFoot')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # ro.inverse_kinematics()
        marker_array = ulink_to_marker_array(ro.ulink)
        pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        rk_api()
    except rospy.ROSInterruptException:
        pass