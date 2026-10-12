import json
import math
import os

import pytest

from ai_nav.plan_logic import load_places, parse_plan, yaw_to_quaternion

PLACES = load_places(os.path.join(os.path.dirname(__file__), '..', 'config', 'places_field.yaml'))


def p(obj):
    return json.dumps(obj)


def test_goto():
    assert parse_plan(p({'action': 'goto', 'targets': ['upper_plaza', 'start']}), PLACES) == \
        ('goto', ['upper_plaza', 'start'])


def test_patrol_expands_loops():
    assert parse_plan(p({'action': 'patrol', 'targets': ['start', 'bottom_right'], 'loops': 2}),
                      PLACES)[1] == ['start', 'bottom_right'] * 2


def test_stop_ignores_targets():
    assert parse_plan(p({'action': 'stop'}), PLACES) == ('stop', [])


@pytest.mark.parametrize('bad', [
    'not json',
    p([1, 2]),
    p({'action': 'fly', 'targets': ['start']}),
    p({'action': 'goto', 'targets': []}),
    p({'action': 'goto', 'targets': ['moon']}),
    p({'action': 'goto', 'targets': ['start'] * 9}),
    p({'action': 'patrol', 'targets': ['start'], 'loops': 10}),
    p({'action': 'patrol', 'targets': ['start'], 'loops': '2'}),
])
def test_rejects(bad):
    with pytest.raises(ValueError):
        parse_plan(bad, PLACES)


def test_quaternion():
    z, w = yaw_to_quaternion(math.pi / 2)
    assert math.isclose(z, math.sqrt(0.5)) and math.isclose(w, math.sqrt(0.5))


# ---------- LiDAR による通路の確認 ----------
import math  # noqa: E402

from ai_nav.plan_logic import aisle_scan_hits  # noqa: E402

SOUTH = {'x': 3.5, 'y': 1.05}


def fake_scan(robot, yaw, obstacles, walls_y=(0.6, 1.5), n=360, rmax=20.0):
    """ロボット位置から 360 本の光線を出し、パレット(軸平行の箱)と通路の壁(y=一定の線)で打ち切る。"""
    ranges = []
    for i in range(n):
        a = yaw + math.radians(i)
        dx, dy = math.cos(a), math.sin(a)
        best = rmax
        for (x0, x1, y0, y1) in obstacles:
            for t in [i2 * 0.01 for i2 in range(1, 2000)]:
                x, y = robot[0] + dx * t, robot[1] + dy * t
                if x0 <= x <= x1 and y0 <= y <= y1:
                    best = min(best, t)
                    break
        ranges.append(best)
    return ranges


def hits(ranges, robot, yaw=0.0):
    return aisle_scan_hits(ranges, 0.0, math.radians(1.0), 0.12, 20.0, robot, yaw, SOUTH)


PALLET = (3.25, 3.75, 0.5, 1.6)                  # 通路を塞ぐパレット(0.5 x 1.1 m)
WALL_N = (2.0, 5.0, 1.5, 1.6)                    # 通路の壁(中心から約 0.45 m)
WALL_S = (2.0, 5.0, 0.5, 0.6)


def test_pallet_in_aisle_is_detected_from_inside_the_aisle():
    r = fake_scan((2.6, 1.05), 0.0, [PALLET, WALL_N, WALL_S])
    assert hits(r, (2.6, 1.05)) >= 3


def test_pallet_is_detected_from_the_west_hall():
    r = fake_scan((0.8, 0.8), 0.0, [PALLET, WALL_N, WALL_S])
    assert hits(r, (0.8, 0.8)) >= 3


def test_empty_aisle_with_walls_has_no_hits():
    r = fake_scan((2.6, 1.05), 0.0, [WALL_N, WALL_S])
    assert hits(r, (2.6, 1.05)) < 3


def test_robot_yaw_is_taken_into_account():
    r = fake_scan((2.6, 1.05), math.pi / 2, [PALLET, WALL_N, WALL_S])   # ロボットは北向き
    assert hits(r, (2.6, 1.05), yaw=math.pi / 2) >= 3
