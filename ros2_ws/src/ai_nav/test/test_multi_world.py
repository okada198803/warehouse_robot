import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ai_nav.route_check import load_map  # noqa: E402

PKG = os.path.join(os.path.dirname(__file__), '..')


def places(name):
    with open(os.path.join(PKG, 'config', f'places_{name}.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)['places']


def test_wide_places_have_lanes_inside_aisle():
    p = places('warehouse_wide')
    assert p['south_aisle']['y'] == 0.8
    e, w = p['south_lane_e'], p['south_lane_w']
    assert e['kind'] == w['kind'] == 'area'
    assert e['y'] > p['south_aisle']['y'] > w['y']          # 東向きは北側、西向きは南側(左側通行)
    assert 0.2 < w['y'] and e['y'] < 1.5 - 0.2               # 壁・棚から 0.2 m 以上離れる


def test_wide_places_otherwise_equal_to_base():
    base, wide = places('warehouse'), places('warehouse_wide')
    assert set(wide) - set(base) == {'south_lane_e', 'south_lane_w'}
    for k in base:
        if k != 'south_aisle':
            assert base[k] == wide[k]


def test_wide_map_differs_only_at_south_gate():
    a, oa, ra = load_map(os.path.join(PKG, 'maps', 'warehouse.yaml'))
    b, ob, rb = load_map(os.path.join(PKG, 'maps', 'warehouse_wide.yaml'))
    assert a.shape == b.shape and ra == rb and list(oa) == list(ob)
    diff = (a != b)
    assert diff.any() and not (diff & b).any()               # 広い側は占有が減るだけ
    rows, cols = diff.nonzero()
    h = a.shape[0]
    xs = oa[0] + (cols + 0.5) * ra
    ys = oa[1] + (h - 1 - rows + 0.5) * ra
    assert xs.min() > 3.3 and xs.max() < 3.7 and ys.min() > -0.1 and ys.max() < 0.7   # 仕切り(x 3.4–3.6, y 0–0.6)の範囲だけ
