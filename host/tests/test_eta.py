import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from eta import LOAD_WAIT, EtaModel, EtaTracker, errors, fit  # noqa: E402


def test_predict_adds_distance_stops_and_cargo_wait():
    m = EtaModel(sec_per_m=3.0, sec_per_stop=2.0, base=1.0)
    assert m.predict(5.0, 2) == 1.0 + 15.0 + 4.0
    assert m.predict(5.0, 2, n_cargo=1) == 20.0 + LOAD_WAIT


def test_fit_recovers_parameters():
    truth = EtaModel(sec_per_m=3.5, sec_per_stop=1.5, base=0.5)
    samples = [(l, n, 0, truth.predict(l, n)) for l, n in [(2, 1), (5, 2), (8, 2), (11, 3), (6, 1), (9, 3)]]
    m = fit(samples)
    assert abs(m.sec_per_m - 3.5) < 1e-6 and abs(m.sec_per_stop - 1.5) < 1e-6 and abs(m.base - 0.5) < 1e-6
    _, mae, mape = errors(m, samples)
    assert mae < 1e-6 and mape < 1e-6


def test_fit_needs_three_samples():
    try:
        fit([(1, 1, 0, 5), (2, 1, 0, 8)])
    except ValueError:
        return
    raise AssertionError('ValueError expected')


def test_tracker_reports_error_once():
    t = EtaTracker()
    assert t.finish(10.0) is None
    t.start(30.0, 100.0)
    pred, meas, err = t.finish(127.0)
    assert (pred, meas, err) == (30.0, 27.0, 3.0)
    assert t.finish(130.0) is None


def test_fit_never_returns_negative_stop_time():
    # 経路長と停止回数が強く相関していても、秒数の係数は負にならない
    samples = [(1.8, 1, 0, 10.9), (3.4, 1, 0, 14.3), (5.6, 2, 0, 22.4), (6.2, 2, 0, 27.4), (9.0, 3, 0, 39.0)]
    m = fit(samples)
    assert m.sec_per_m >= 0 and m.sec_per_stop >= 0 and m.base >= 0


def test_fit_learns_cargo_wait_when_cargo_samples_exist():
    truth = EtaModel(sec_per_m=4.0, sec_per_stop=1.0, base=0.5, load_wait=9.0)
    samples = [(l, n, c, truth.predict(l, n, c)) for l, n, c in
               [(3, 1, 0), (6, 2, 0), (9, 2, 0), (6, 2, 1), (12, 3, 1), (7, 2, 1), (10, 3, 1)]]
    m = fit(samples)
    assert abs(m.load_wait - 9.0) < 1e-6 and abs(m.sec_per_m - 4.0) < 1e-6


def test_fit_keeps_default_cargo_wait_without_cargo_samples():
    samples = [(l, n, 0, 3.0 * l + n) for l, n in [(2, 1), (5, 2), (8, 2), (11, 3)]]
    assert fit(samples).load_wait == LOAD_WAIT
