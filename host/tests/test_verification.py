"""検証ランナー(V1/V4)の部品のテスト。Ollama 不要。"""
import measure_llm
from experience import Experience


def _exp(similar):
    e = Experience(None, k=2, read_only=False, similar=similar)
    e.items = [
        {'blocked': ['south_aisle'], 'instruction': 'a', 'targets': ['x'], 'length': 1.0},
        {'blocked': ['south_aisle', 'middle_aisle'], 'instruction': 'b', 'targets': ['y'], 'length': 2.0},
    ]
    return e


def test_exact_match_has_priority():
    e = _exp(True)
    hits = e.retrieve(['south_aisle'], similar=True)
    assert [h['instruction'] for h in hits] == ['a']
    assert not hits[0].get('similar')


def test_no_similar_by_default():
    assert _exp(False).retrieve(['north_aisle', 'south_aisle']) == []


def test_similar_ranks_by_overlap_and_flags():
    hits = _exp(True).retrieve(['south_aisle', 'north_aisle'], similar=True)
    assert [h['instruction'] for h in hits] == ['a', 'b']   # 1/2 > 1/3
    assert all(h['similar'] for h in hits)


def test_similar_zero_overlap_excluded():
    assert _exp(True).retrieve(['north_aisle'], similar=True) == []


def test_v4_scenarios_shape():
    sc = measure_llm.v4_scenarios()
    assert len(sc) == 24
    assert len({s[0] for s in sc}) == 24
    assert all(s[2][0][0] == 'user' for s in sc)


def _planner():
    from planner import Planner
    return Planner('warehouse', lambda m, s: (_ for _ in ()).throw(AssertionError('LLM を呼ばない')))


def test_clear_paraphrases_handled_by_rule():
    for t in ['南通路の障害物はなくなった', '南通路はもう空いている', '南通路は片付いた', '南通路の荷物を撤去した']:
        p = _planner()
        p.blocked = {'south_aisle'}
        d = p.handle_user(t)
        assert d.source == 'clear' and 'south_aisle' not in p.blocked, t


def test_negated_or_pending_clear_is_not_clear():
    for t in ['南通路はまだ片付いていない', '南通路は空いていない', '南通路は通れなくなった', '南通路は通れる?', '南通路は片付いたかな']:
        assert _planner()._clear_targets(t) == [], t


def _llm_returning(**out):
    base = {'action': 'info', 'targets': [], 'blocked_add': [], 'blocked_clear': [], 'reply': 'x'}
    base.update(out)
    return lambda m, s: dict(base)


def test_llm_clear_marks_cleared_for_lidar_check():
    from planner import Planner
    p = Planner('warehouse', _llm_returning(blocked_clear=['south_aisle']), clear_rules=False)
    p.blocked = {'south_aisle'}
    p.handle_user('南通路はもう大丈夫になった')
    assert p.blocked == set() and p.cleared == {'south_aisle'}


def test_llm_clear_ignored_when_negated():
    from planner import Planner
    p = Planner('warehouse', _llm_returning(blocked_clear=['south_aisle']), clear_rules=False)
    p.blocked = {'south_aisle'}
    p.handle_user('南通路はまだ片付いていない')
    assert p.blocked == {'south_aisle'} and p.cleared == set()


def test_v4x_scenarios_shape():
    assert len(measure_llm.v4x_scenarios()) == 13
    assert 'ていない' in measure_llm.PARAPHRASES_N[0]


def test_clear_hint_in_prompt():
    from planner import Planner
    assert '解除の言い方の例' in Planner('warehouse', None, clear_hint=True).system
    assert '解除の言い方の例' not in Planner('warehouse', None).system


def test_llm_clear_ignored_in_report_reply():
    from planner import Planner
    p = Planner('warehouse', _llm_returning(action='goto', targets=['shipping'], blocked_clear=['middle_aisle']))
    p.blocked = {'south_aisle', 'middle_aisle'}
    p.handle_blocked({'state': 'blocked', 'target': 'shipping', 'remaining': ['shipping'],
                      'near_aisle': 'middle_aisle', 'cause': 'obstacle', 'pose': [1.5, 2.5]})
    assert 'middle_aisle' in p.blocked


def test_report_reply_uses_loose_schema():
    from planner import Planner
    p = Planner('warehouse', None)
    assert 'blocked_clear' in p.schema['required']
    assert 'blocked_clear' not in p.schema_report['required']
