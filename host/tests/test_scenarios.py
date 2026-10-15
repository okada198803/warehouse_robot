"""複数ロボットのシナリオ定義(auto_multi.SCENARIOS)の整合性。ROS・Gazebo は使わない。"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'ros2_ws', 'src', 'ai_nav'))

import auto_multi as M  # noqa: E402
from fleet import conflict, intent_of  # noqa: E402
from planner import Planner  # noqa: E402


def planners(scn, world='warehouse'):
    M.set_scenario(scn)
    out = {}
    for ns, c in M.ROBOTS.items():
        p = Planner(world, M.make_stub(c['dest']))
        if c.get('pose'):
            p.pose = tuple(c['pose'][:2])
        else:
            p.last_arrived = c['home']
        out[ns] = p
    return out


def intents(scn, world='warehouse'):
    pl = planners(scn, world)
    return {ns: intent_of(pl[ns], ns, M.ROBOTS[ns]['dest']) for ns in pl}


def teardown_function(_):
    M.set_scenario('opp')


def test_opp_unchanged():
    M.set_scenario('opp')
    assert M.POSES['r1'] == (0.8, 0.8, 0.0) and M.POSES['r2'][:2] == (6.2, 0.8)
    assert M.ROBOTS['r1'] == {'home': 'receiving', 'dest': 'shipping'}
    assert M.ROBOTS['r2'] == {'home': 'shipping', 'dest': 'receiving'}


def test_opp_conflicts_on_south():
    it = intents('opp')
    assert it['r1']['aisle'] == it['r2']['aisle'] == 'south_aisle' and conflict(it['r1'], it['r2'])


def test_same_direction_shares_south_without_conflict():
    it = intents('same')
    assert it['r1']['aisle'] == it['r2']['aisle'] == 'south_aisle'
    assert it['r1']['dir'] == it['r2']['dir'] == 'E' and not conflict(it['r1'], it['r2'])


def test_cross_uses_different_destinations_through_middle():
    it = intents('cross')
    assert {M.ROBOTS['r1']['dest'], M.ROBOTS['r2']['dest']} == {'charger', 'shipping'}
    assert it['r1']['aisle'] == it['r2']['aisle'] == 'middle_aisle' and not conflict(it['r1'], it['r2'])


def test_wait_robot_starts_inside_south_aisle_and_departs_later():
    M.set_scenario('wait')
    assert M.ROBOTS['r2']['home'] == 'south_aisle' and M.ROBOTS['r2']['delay'] > 0
    assert 'delay' not in M.ROBOTS['r1']
    assert M.POSES['r2'][:2] == (3.5, 1.05)


def test_poses_match_start_places():
    for scn in ('same', 'cross', 'wait'):
        M.set_scenario(scn)
        pl = planners(scn)
        for ns, c in M.ROBOTS.items():
            if c.get('home'):
                pt = pl[ns].places[c['home']]
                assert M.POSES[ns][:2] == (pt['x'], pt['y'])


def test_south_use_in_wide_world_for_lane_rule():
    pl = planners('same', 'warehouse_wide')
    assert pl['r2'].handle_user(M.TEXT).plan['targets'] == ['shipping']     # 通路の経由地は不要
    assert M.uses_south(pl['r2'], 'shipping')                                  # だが経路は南通路を通る
    pl = planners('cross', 'warehouse_wide')
    assert not any(M.uses_south(p, M.ROBOTS[ns]['dest']) for ns, p in pl.items())
