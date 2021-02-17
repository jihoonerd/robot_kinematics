from viz.utils import ulink_to_marker_array
from rk.robot_preset.biped_robot import biped_ro
import copy


def test_ulink_to_marker_array():
    ro = copy.deepcopy(biped_ro)
    marker_arr = ulink_to_marker_array(ro.ulink)

    assert len(marker_arr.markers) == 15
    assert len(marker_arr.markers[7].points) == 7
    assert len(marker_arr.markers[14].points) == 7
    assert len(marker_arr.markers[0].points) == 0
    assert len(marker_arr.markers[6].points) == 0
    assert len(marker_arr.markers[13].points) == 0

