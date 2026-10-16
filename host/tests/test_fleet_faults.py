"""通信の欠落・遅延(C3x・C3h・C3d)と、2 層(C4)の補助関数のテスト。"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fleet import PeerChannel, aisle_at, central_assign  # noqa: E402
from planner import Planner  # noqa: E402


def make(home):
    p = Planner('warehouse', lambda m, s: {})
    p.last_arrived = home
    return p


class FakeTopic:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


def chan(drop_p, delay=0.0, seed=1):
    c = PeerChannel.__new__(PeerChannel)
    c.ns, c.topic = 'r1', FakeTopic()
    c.reset('t', drop_p, delay, seed)
    return c


def test_drop_all_sends_nothing():
    c = chan(1.0)
    c.announce({'from': 'r1', 'aisle': 'south_aisle', 'dir': 'E'}, repeat=2, interval=0.0)
    assert c.dropped == 1


def test_half_drop_is_reproducible():
    a, b = chan(0.5, seed=7), chan(0.5, seed=7)
    for c in (a, b):
        for _ in range(20):
            c.announce({'from': 'r1', 'aisle': None, 'dir': 'E'}, repeat=1, interval=0.0)
    assert a.dropped == b.dropped and 0 < a.dropped < 20


def test_default_reset_keeps_old_behaviour():
    c = chan(0.0)
    assert c.drop_p == 0.0 and c.delay == 0.0


def test_aisle_at():
    p = make('receiving')
    assert aisle_at(p, (3.5, 1.05)) == 'south_aisle'
    assert aisle_at(p, (3.0, 2.5)) == 'middle_aisle'
    assert aisle_at(p, (0.8, 0.8)) is None


def test_central_assign_rejects_occupied_aisle_then_accepts():
    ps = {'r1': make('receiving')}
    answers = iter([{'route': {'r1': 'south_aisle'}, 'reason': ''},
                    {'route': {'r1': 'middle_aisle'}, 'reason': ''}])
    seen = []

    def llm(m, s):
        seen.append(m[1]['content'])
        return next(answers)
    avoid, info = central_assign(llm, ps, {'r1': 'shipping'}, occupied={'south_aisle': 'r2'})
    assert info['valid'] and info['calls'] == 2
    assert 'south_aisle' in avoid['r1']
    assert '応答のないロボット r2' in seen[0]


def test_central_assign_without_occupied_unchanged():
    ps = {'r1': make('receiving')}
    avoid, info = central_assign(lambda m, s: {'route': {'r1': 'south_aisle'}, 'reason': ''}, ps, {'r1': 'shipping'})
    assert info['valid'] and info['calls'] == 1
