import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fleet import (central_assign, central_prompt, check_assignment, conflict,  # noqa: E402
                   intent_of, resolve)
from planner import Planner  # noqa: E402

DESTS = {'r1': 'shipping', 'r2': 'receiving'}


def make(ns, home):
    p = Planner('warehouse', lambda m, s: {})
    p.last_arrived = home
    return p


def pair():
    return {'r1': make('r1', 'receiving'), 'r2': make('r2', 'shipping')}


def test_default_intents_conflict_on_south_aisle():
    ps = pair()
    a = intent_of(ps['r1'], 'r1', 'shipping')
    b = intent_of(ps['r2'], 'r2', 'receiving')
    assert a['aisle'] == b['aisle'] == 'south_aisle'
    assert (a['dir'], b['dir']) == ('E', 'W')
    assert conflict(a, b)


def test_same_direction_is_not_conflict():
    a = {'from': 'r1', 'aisle': 'south_aisle', 'dir': 'E'}
    b = {'from': 'r2', 'aisle': 'south_aisle', 'dir': 'E'}
    assert not conflict(a, b)


def test_resolve_yields_to_higher_priority():
    ps = pair()
    it1, av1, ok1 = resolve(ps['r1'], 'r1', 'shipping', [])
    assert (it1['aisle'], av1, ok1) == ('south_aisle', set(), True)
    it2, av2, ok2 = resolve(ps['r2'], 'r2', 'receiving', [it1])
    assert ok2 and av2 == {'south_aisle'} and it2['aisle'] == 'middle_aisle'


def test_intent_does_not_register_blocked():
    ps = pair()
    intent_of(ps['r2'], 'r2', 'receiving', avoid={'south_aisle'})
    assert ps['r2'].blocked == set()


def test_check_assignment_detects_remaining_conflict():
    ps = pair()
    problems, _ = check_assignment(ps, DESTS, {})
    assert problems
    problems, _ = check_assignment(ps, DESTS, {'r2': {'south_aisle'}})
    assert not problems


def test_central_assign_accepts_valid_answer():
    ps = pair()
    avoid, info = central_assign(lambda m, s: {'avoid': {'r1': [], 'r2': ['south_aisle']}, 'reason': 'x'}, ps, DESTS)
    assert info['valid'] and info['calls'] == 1 and avoid['r2'] == {'south_aisle'}


def test_central_assign_retries_then_falls_back():
    ps = pair()
    avoid, info = central_assign(lambda m, s: {'avoid': {'r1': [], 'r2': []}, 'reason': 'x'}, ps, DESTS)
    assert not info['valid'] and info['calls'] == 3
    assert avoid == {'r1': set(), 'r2': set()}


def test_central_assign_recovers_after_feedback():
    ps = pair()
    answers = iter([{'avoid': {'r1': [], 'r2': []}, 'reason': 'a'},
                    {'avoid': {'r1': ['south_aisle'], 'r2': []}, 'reason': 'b'}])
    avoid, info = central_assign(lambda m, s: next(answers), ps, DESTS)
    assert info['valid'] and info['calls'] == 2 and avoid['r1'] == {'south_aisle'}


def test_central_assign_rejects_all_aisles_avoided():
    ps = pair()
    allp = ['south_aisle', 'middle_aisle', 'north_aisle']
    avoid, info = central_assign(lambda m, s: {'avoid': {'r1': [], 'r2': allp}, 'reason': 'x'}, ps, DESTS)
    assert not info['valid']


def test_prompt_mentions_each_robot():
    t = central_prompt(pair(), DESTS)
    assert 'r1:' in t and 'r2:' in t and '東向き' in t and '西向き' in t
