from robot_config.biped_robot import biped_ulink
from robot_config.utils import find_mother


def test_find_mother():
    mother_found_ulink = find_mother(biped_ulink, 1)
    assert mother_found_ulink[1].mother == 0
    assert mother_found_ulink[2].mother == 1
    assert mother_found_ulink[3].mother == 2
    assert mother_found_ulink[4].mother == 3
    assert mother_found_ulink[5].mother == 4
    assert mother_found_ulink[6].mother == 5
    assert mother_found_ulink[7].mother == 6
    assert mother_found_ulink[8].mother == 1
    assert mother_found_ulink[9].mother == 8
    assert mother_found_ulink[10].mother == 9
    assert mother_found_ulink[11].mother == 10
    assert mother_found_ulink[12].mother == 11
    assert mother_found_ulink[13].mother == 12