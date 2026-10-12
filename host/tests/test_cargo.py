"""荷物の搬送(--cargo): 指差しによる指示語の解決、荷物 → 目的地の計画、既定動作の不変。"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from planner import Planner  # noqa: E402
from test_planner import ScriptedLLM, goto  # noqa: E402


def info(reply='了解'):
    return {'action': 'info', 'targets': [], 'reply': reply}


def test_cargo_places_hidden_by_default():
    p = Planner('warehouse', ScriptedLLM(goto('shipping')))
    assert not any(v.get('kind') == 'cargo' for v in p.places.values())
    assert '荷物置き場' not in p.system and 'cargo_' not in p.system


def test_cargo_places_visible_with_option():
    p = Planner('warehouse', ScriptedLLM(goto('shipping')), cargo=True)
    assert 'cargo_center' in p.places and '荷物置き場' in p.system
    assert 'cargo_center' in p.schema['properties']['targets']['items']['enum']


def test_demonstrative_with_pointing_goes_cargo_then_destination():
    llm = ScriptedLLM(goto('cargo_center', 'shipping'))
    p = Planner('warehouse', llm, cargo=True)
    d = p.handle_user('あれを出荷口へ運んで', pointed='cargo_center')
    # 中央の荷物(1.0, 2.5)からは、中央通路が出荷口までの最短(経路ごとに最短を補う)
    assert d.plan['targets'] == ['cargo_center', 'middle_aisle', 'shipping']
    assert '[指差し: 中央の荷物(cargo_center)]' in llm.calls[0][-1]['content']
    assert d.reply == '中央の荷物を積載し、中央通路経由で出荷口へ運びます'


def test_pointing_overrides_wrong_llm_cargo():
    # LLM が別の荷物を選んでも、指差しで確定した荷物にそろえる
    p = Planner('warehouse', ScriptedLLM(goto('cargo_south', 'shipping')), cargo=True)
    d = p.handle_user('あれを出荷口へ', pointed='cargo_north')
    assert d.plan['targets'][0] == 'cargo_north' and d.plan['targets'][-1] == 'shipping'
    assert any('補正' in line for line in d.log)


def test_llm_that_drops_cargo_is_corrected():
    p = Planner('warehouse', ScriptedLLM(goto('shipping')), cargo=True)
    d = p.handle_user('あれを出荷口へ', pointed='cargo_south')
    assert d.plan['targets'][0] == 'cargo_south' and d.plan['targets'][-1] == 'shipping'


def test_llm_that_answers_info_is_corrected_when_pointing_and_destination_given():
    p = Planner('warehouse', ScriptedLLM(info('どれですか')), cargo=True)
    d = p.handle_user('それを出荷口へ運んで', pointed='cargo_center')
    assert d.plan and d.plan['targets'][0] == 'cargo_center' and d.plan['targets'][-1] == 'shipping'


def test_demonstrative_without_pointing_asks_back_without_calling_llm():
    llm = ScriptedLLM(goto('shipping'))
    p = Planner('warehouse', llm, cargo=True)
    d = p.handle_user('あれを出荷口へ運んで', pointed=None)
    assert d.plan is None and d.source == 'ask' and 'どの荷物' in d.reply
    assert llm.calls == []


def test_named_cargo_without_pointing():
    p = Planner('warehouse', ScriptedLLM(goto('cargo_south', 'shipping')), cargo=True)
    d = p.handle_user('南の荷物を出荷口へ運んで')
    assert d.plan['targets'] == ['cargo_south', 'south_aisle', 'shipping']


def test_pointing_ignored_without_demonstrative():
    llm = ScriptedLLM(goto('shipping'))
    p = Planner('warehouse', llm, cargo=True)
    d = p.handle_user('出荷口へ行って', pointed='cargo_north')
    assert d.plan['targets'] == ['south_aisle', 'shipping']
    assert '指差し' not in llm.calls[0][-1]['content']


def test_pointing_ignored_when_cargo_mode_off():
    llm = ScriptedLLM(goto('shipping'))
    p = Planner('warehouse', llm)
    d = p.handle_user('あれを出荷口へ', pointed='cargo_north')
    assert d.plan['targets'] == ['south_aisle', 'shipping']


def test_cargo_route_avoids_blocked_aisle():
    p = Planner('warehouse', ScriptedLLM(goto('cargo_south', 'shipping')), cargo=True)
    p.blocked = {'south_aisle'}
    d = p.handle_user('あれを出荷口へ', pointed='cargo_south')
    assert 'south_aisle' not in d.plan['targets'] and 'middle_aisle' in d.plan['targets']


def test_replan_while_carrying_keeps_destination():
    llm = ScriptedLLM(goto('cargo_center', 'shipping'), goto('middle_aisle', 'shipping'))
    p = Planner('warehouse', llm, cargo=True)
    p.handle_user('あれを出荷口へ', pointed='cargo_center')
    p.on_status({'state': 'arrived', 'target': 'cargo_center'})
    p.on_status({'state': 'loaded', 'target': 'cargo_center'})
    assert p.carrying == 'cargo_center'
    d = p.handle_blocked({'state': 'blocked', 'near_aisle': 'south_aisle', 'target': 'shipping',
                          'remaining': ['south_aisle', 'shipping'], 'pose': [2.9, 1.05]})
    assert d.plan['targets'] == ['middle_aisle', 'shipping']
    assert '積載中: 中央の荷物' in llm.calls[-1][-1]['content']
    p.on_status({'state': 'unloaded', 'target': 'cargo_center'})
    assert p.carrying is None


def test_cargo_plans_are_not_saved_to_experience():
    class Exp:
        similar = False

        def __init__(self):
            self.items = []

        def retrieve(self, blocked, similar=False):
            return []

        def add(self, **kw):
            self.items.append(kw)

    exp = Exp()
    p = Planner('warehouse', ScriptedLLM(goto('cargo_center', 'shipping')), experience=exp, cargo=True)
    p.handle_user('あれを出荷口へ', pointed='cargo_center')
    p.on_status({'state': 'done'})
    assert exp.items == []


def test_existing_behavior_unchanged_for_plain_instruction_in_cargo_mode():
    p = Planner('warehouse', ScriptedLLM(goto('shipping')), cargo=True)
    d = p.handle_user('出荷口へ荷物を運んで')
    assert d.plan['targets'] == ['south_aisle', 'shipping']


def test_input_without_place_name_is_not_saved_to_experience():
    class Exp:
        similar = False

        def __init__(self):
            self.items = []

        def retrieve(self, blocked, similar=False):
            return []

        def add(self, **kw):
            self.items.append(kw)

    exp = Exp()
    p = Planner('warehouse', ScriptedLLM(goto('charger')), experience=exp)
    p.handle_user('python pointing_cam.py')
    p.on_status({'state': 'done'})
    assert exp.items == []
    q = Planner('warehouse', ScriptedLLM(goto('charger')), experience=exp)
    q.handle_user('充電ステーションへ行って')
    q.on_status({'state': 'done'})
    assert len(exp.items) == 1
