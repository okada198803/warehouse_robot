import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ai_nav.cargo_logic import (BOUNDS, CargoTracker, cargo_ids, drop_xy,  # noqa: E402
                                lerp)
from ai_nav.plan_logic import load_places  # noqa: E402

PLACES = load_places(os.path.join(os.path.dirname(__file__), '..', 'config',
                                  'places_warehouse.yaml'))


def arrived(t):
    return {'state': 'arrived', 'target': t}


def test_cargo_places_defined():
    ids = cargo_ids(PLACES)
    assert sorted(ids) == ['cargo_center', 'cargo_north', 'cargo_south']
    for pid in ids:
        b = PLACES[pid]['box']
        # 箱は到着地点の 0.5〜0.6 m 先(ホール車線から外した位置)
        d = math.dist((PLACES[pid]['x'], PLACES[pid]['y']), (b['x'], b['y']))
        assert 0.5 - 1e-9 <= d <= 0.6 + 1e-9


def test_boxes_stay_out_of_aisle_scan_bands():
    # 通路の事前確認の帯は x = 通路中心 ± 1.2 m(2.3 m 以上)。箱はその外にある
    for pid in cargo_ids(PLACES):
        assert PLACES[pid]['box']['x'] + 0.15 < 3.5 - 1.2


def test_load_then_unload():
    t = CargoTracker(PLACES)
    assert t.update(arrived('cargo_center')) == [('loaded', 'cargo_center')]
    assert t.update(arrived('south_aisle')) == []
    assert t.update(arrived('shipping')) == []
    assert t.update({'state': 'done'}) == [('unloaded', 'cargo_center', 'shipping')]
    assert t.loaded is None


def test_no_unload_when_plan_ends_at_cargo():
    t = CargoTracker(PLACES)
    t.update(arrived('cargo_north'))
    assert t.update({'state': 'done'}) == []
    assert t.loaded == 'cargo_north'
    # 次の計画で目的地へ着けば降ろす
    t.update(arrived('shipping'))
    assert t.update({'state': 'done'}) == [('unloaded', 'cargo_north', 'shipping')]


def test_blocked_keeps_cargo_loaded():
    t = CargoTracker(PLACES)
    t.update(arrived('cargo_south'))
    t.update({'state': 'blocked', 'target': 'shipping'})
    assert t.loaded == 'cargo_south'
    t.update(arrived('middle_aisle'))
    t.update(arrived('shipping'))
    assert t.update({'state': 'done'}) == [('unloaded', 'cargo_south', 'shipping')]


def test_plain_goto_does_nothing():
    t = CargoTracker(PLACES)
    assert t.update(arrived('shipping')) == []
    assert t.update({'state': 'done'}) == []


def test_second_cargo_is_ignored_while_loaded():
    t = CargoTracker(PLACES)
    t.update(arrived('cargo_south'))
    assert t.update(arrived('cargo_north')) == []
    assert t.loaded == 'cargo_south'


def test_reset():
    t = CargoTracker(PLACES)
    t.update(arrived('cargo_south'))
    t.reset()
    assert t.loaded is None


def test_drop_xy_inside_floor():
    for pid, p in PLACES.items():
        x, y = drop_xy(p)
        assert BOUNDS[0] <= x <= BOUNDS[1] and BOUNDS[2] <= y <= BOUNDS[3]
    x, y = drop_xy(PLACES['shipping'])
    assert abs(x - 6.75) < 1e-9 and abs(y - 0.8) < 1e-9   # 東の壁の手前に収まる


def test_lerp_clamps():
    assert lerp((0, 0, 0), (2, 4, 6), 0.5) == (1, 2, 3)
    assert lerp((0, 0, 0), (2, 4, 6), 2.0) == (2, 4, 6)
    assert lerp((0, 0, 0), (2, 4, 6), -1) == (0, 0, 0)
