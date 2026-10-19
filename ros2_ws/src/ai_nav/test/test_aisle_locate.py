import os

import pytest

from ai_nav.aisle_locate import locate_aisle, should_retry_abort
from ai_nav.plan_logic import load_places

PLACES = load_places(os.path.join(os.path.dirname(__file__), '..', 'config', 'places_warehouse.yaml'))

# 複数台の失敗の型 C(6 件)の最初の停止位置。元の規則では全て unlocated(docs/perception/R0_R6_ログ再解析.md)
TYPE_C_STOPS = [(2.7, 1.22), (3.1, 1.0), (2.1, 1.12), (3.7, 0.46), (3.25, 0.63), (2.56, 1.42)]


@pytest.mark.parametrize('pose', TYPE_C_STOPS)
def test_type_c_stops_are_in_south_aisle(pose):
    assert locate_aisle(pose, PLACES) == 'south_aisle'


def test_type_b_stop_near_north_wall_of_south_aisle():
    assert locate_aisle((2.87, 1.44), PLACES) == 'south_aisle'


@pytest.mark.parametrize('pose,aisle', [((3.74, 2.5), 'middle_aisle'), ((3.5, 3.95), 'north_aisle'),
                                        ((3.5, 2.2), 'middle_aisle'), ((3.5, 1.45), 'south_aisle')])
def test_other_aisles(pose, aisle):
    assert locate_aisle(pose, PLACES) == aisle


@pytest.mark.parametrize('pose', [(0.93, 1.01), (6.2, 0.8), (1.03, 1.0), (1.82, 2.1), (0.8, 0.8)])
def test_halls_and_docks_are_not_aisles(pose):
    assert locate_aisle(pose, PLACES) is None


def test_none_pose():
    assert locate_aisle(None, PLACES) is None


def test_abort_retry():
    assert should_retry_abort(0, 1) is True
    assert should_retry_abort(1, 1) is False
    assert should_retry_abort(0, 0) is False
