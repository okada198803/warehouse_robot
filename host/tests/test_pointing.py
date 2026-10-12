import json
import os
import sys
from types import SimpleNamespace as P

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from pointing import (Calibration, PointingSelector, clear_selection, is_pointing,  # noqa: E402
                      pointing_feature, read_selection, write_selection)


def hand(index_dx=0.0, curled=True, index_out=True):
    """手首 (0.5, 0.8)、中指の付け根 (0.5, 0.6)(手の大きさ 0.2)の合成ランドマーク 21 点。"""
    lm = [P(x=0.5, y=0.8) for _ in range(21)]
    lm[0] = P(x=0.5, y=0.8)
    lm[5] = P(x=0.45, y=0.62)
    lm[9] = P(x=0.5, y=0.6)
    # 人差し指: 第 2 関節 6、指先 8
    lm[6] = P(x=0.45 + index_dx * 0.05, y=0.55)
    lm[8] = P(x=0.5 + index_dx * 0.2, y=0.35 if index_out else 0.7)
    for pip, tip in ((10, 12), (14, 16), (18, 20)):
        lm[pip] = P(x=0.5, y=0.55)
        lm[tip] = P(x=0.5, y=0.7 if curled else 0.3)    # 曲げている: 指先が手首に近い
    return lm


def test_is_pointing_only_index_extended():
    assert is_pointing(hand())
    assert not is_pointing(hand(curled=False))        # 全部伸びている(パー)
    assert not is_pointing(hand(index_out=False))     # 握りこぶし


def test_feature_sign_and_scale():
    assert pointing_feature(hand(index_dx=0.0)) == 0.0
    assert pointing_feature(hand(index_dx=1.0)) > 0
    assert pointing_feature(hand(index_dx=-1.0)) < 0
    assert abs(pointing_feature(hand(index_dx=1.0)) - 1.0) < 1e-6   # 0.2 / 手の大きさ 0.2


def test_classify_three_directions():
    c = Calibration()
    assert c.classify(-1.0) == ('cargo_north', 'ok')
    assert c.classify(0.05) == ('cargo_center', 'ok')
    assert c.classify(0.95) == ('cargo_south', 'ok')


def test_classify_ambiguous_between_and_out_of_range():
    c = Calibration()
    assert c.classify(-0.5) == (None, 'ambiguous')
    assert c.classify(0.5) == (None, 'ambiguous')
    assert c.classify(2.5) == (None, 'out')
    assert c.classify(-2.5) == (None, 'out')


def test_fit_uses_median_and_orders_by_samples():
    c = Calibration.fit({'cargo_north': [-0.7, -0.6, 5.0], 'cargo_center': [0.1, 0.0, 0.2],
                         'cargo_south': [0.9, 1.0, 0.8]})
    assert c.centers['cargo_north'] == -0.6
    assert c.classify(-0.65)[0] == 'cargo_north'


def test_calibration_roundtrip(tmp_path):
    p = tmp_path / 'c.json'
    Calibration({'cargo_north': -0.8, 'cargo_center': 0.1, 'cargo_south': 0.9}).save(str(p))
    assert Calibration.load(str(p)).centers['cargo_north'] == -0.8
    assert Calibration.load(str(tmp_path / 'none.json')).centers['cargo_center'] == 0.0


def test_dwell_confirms_once():
    s = PointingSelector(Calibration(), dwell=1.0)
    assert s.update(-1.0, 0.0) is None
    assert s.update(-0.95, 0.5) is None
    assert s.update(-1.05, 1.0) == 'cargo_north'
    assert s.update(-1.0, 1.2) is None             # 確定は 1 回だけ
    assert s.current(2.0) == 'cargo_north'


def test_dwell_resets_on_change_ambiguous_or_no_hand():
    s = PointingSelector(Calibration(), dwell=1.0)
    s.update(-1.0, 0.0)
    s.update(1.0, 0.6)                              # 別の方向に変わる
    assert s.update(1.0, 1.3) is None               # 変わった時点から数え直す
    assert s.update(1.0, 1.7) == 'cargo_south'
    s2 = PointingSelector(Calibration(), dwell=1.0)
    s2.update(-1.0, 0.0)
    s2.update(-0.5, 0.6)                            # 曖昧
    assert s2.status == 'ambiguous' and s2.candidate is None
    s2.update(None, 0.8)                            # 手が消える
    assert s2.update(-1.0, 1.5) is None


def test_confirmed_expires():
    s = PointingSelector(Calibration(), dwell=1.0, ttl=10.0)
    s.update(0.0, 0.0)
    s.update(0.0, 1.0)
    assert s.current(5.0) == 'cargo_center'
    assert s.current(12.0) is None


def test_selection_file_ttl(tmp_path):
    p = str(tmp_path / 'sel.json')
    write_selection('cargo_south', p, now=100.0)
    assert read_selection(p, ttl=10, now=105.0) == 'cargo_south'
    assert read_selection(p, ttl=10, now=111.0) is None
    clear_selection(p)
    assert read_selection(p, now=101.0) is None
    with open(p, 'w') as f:
        f.write('broken')
    assert read_selection(p, now=101.0) is None
