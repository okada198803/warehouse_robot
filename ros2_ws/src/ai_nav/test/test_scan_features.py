import json
import math
import os

import numpy as np

from ai_nav.plan_logic import aisle_scan_hits, load_places
from ai_nav.scan_features import GX, GY, N_FEATURES, Recognizer, features

PLACES = load_places(os.path.join(os.path.dirname(__file__), '..', 'config', 'places_warehouse.yaml'))
A = PLACES['south_aisle']


def scan_with_point(robot, yaw, target):
    """target の 1 点だけが返る 360 本のスキャン。"""
    ranges = [float('inf')] * 360
    ang = math.atan2(target[1] - robot[1], target[0] - robot[0]) - yaw
    i = int(round((ang % (2 * math.pi)) / (2 * math.pi / 360))) % 360
    ranges[i] = math.dist(robot, target)
    return ranges


def test_feature_length_and_cell():
    robot, yaw = (A['x'] - 1.0, A['y']), 0.0
    target = (A['x'] + 0.5, A['y'] + 0.05)
    f = features(scan_with_point(robot, yaw, target), 0.0, 2 * math.pi / 360, 0.12, 20.0, robot, yaw, A)
    assert len(f) == N_FEATURES
    assert sum(f[:-4]) == 1.0
    ix = int((0.5 - GX[0]) // ((GX[1] - GX[0]) / GX[2]))
    iy = int((0.05 - GY[0]) // ((GY[1] - GY[0]) / GY[2]))
    assert f[iy * GX[2] + ix] == 1.0
    assert abs(f[-4] - (-1.0 / 1.6)) < 1e-9


def test_point_outside_grid_is_ignored():
    robot, yaw = (A['x'], A['y']), 0.0
    f = features(scan_with_point(robot, yaw, (A['x'] + 3.0, A['y'])), 0.0, 2 * math.pi / 360, 0.12, 20.0, robot, yaw, A)
    assert sum(f[:-4]) == 0.0


def test_recognizer_matches_numpy(tmp_path):
    rng = np.random.default_rng(0)
    d, h = N_FEATURES, 5
    m = {'mean': rng.normal(size=d).tolist(), 'std': (rng.random(d) + 0.5).tolist(),
         'w1': rng.normal(size=(h, d)).tolist(), 'b1': rng.normal(size=h).tolist(),
         'w2': rng.normal(size=h).tolist(), 'b2': 0.3}
    p = tmp_path / 'r.json'
    p.write_text(json.dumps(m))
    rec = Recognizer(str(p))
    x = rng.normal(size=d)
    xs = (x - np.array(m['mean'])) / np.array(m['std'])
    hh = np.maximum(0, np.array(m['w1']) @ xs + np.array(m['b1']))
    want = 1 / (1 + np.exp(-(hh @ np.array(m['w2']) + m['b2'])))
    assert abs(rec.prob(x.tolist()) - want) < 1e-9
