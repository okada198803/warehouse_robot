import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from threshold_select import leave_one_group_out, select_threshold  # noqa: E402


def test_separable_picks_middle_of_gap():
    s = select_threshold(pos=[17, 22, 54], neg=[0, 0, 15])
    assert s.separable and (s.miss, s.false) == (0.0, 0.0)
    assert s.threshold == 16 and s.margin == 2


def test_wide_gap_has_larger_margin_than_narrow():
    wide = select_threshold([30, 40], [0, 5])
    narrow = select_threshold([17, 40], [0, 15])
    assert wide.margin > narrow.margin


def test_overlap_minimises_total_error():
    s = select_threshold(pos=[5, 8, 9, 20], neg=[0, 1, 6, 7])
    assert not s.separable
    assert s.miss + s.false <= 0.5


def test_requires_both_classes():
    for args in (([], [1]), ([1], [])):
        try:
            select_threshold(*args)
        except ValueError:
            continue
        raise AssertionError('ValueError expected')


def test_leave_one_group_out_counts_heldout_errors():
    recs = [('a', True, 20), ('a', False, 0), ('b', True, 25), ('b', False, 2), ('c', True, 22), ('c', False, 1)]
    r = leave_one_group_out(recs)
    assert r['miss'] == 0 and r['false'] == 0 and r['positives'] == 3 and r['negatives'] == 3
