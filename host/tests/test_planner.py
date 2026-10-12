"""再計画ループのテスト(LLM は台本どおりに応答する偽物に置き換える)。"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from planner import Planner  # noqa: E402


class ScriptedLLM:
    def __init__(self, *responses):
        self.responses = list(responses)
        self.calls = []

    def __call__(self, messages, schema):
        self.calls.append(messages)
        return self.responses.pop(0) if len(self.responses) > 1 else self.responses[0]


def goto(*targets, reply='了解'):
    return {'action': 'goto', 'targets': list(targets), 'reply': reply}


def blocked_status(aisle, remaining, pose):
    return {'state': 'blocked', 'near_aisle': aisle, 'remaining': remaining,
            'target': remaining[0], 'pose': pose}


def test_direct_plan_passes():
    p = Planner('warehouse', ScriptedLLM(goto('shipping')))
    d = p.handle_user('出荷口へ荷物を運んで')
    # 検証で使う最短経路(南通路)を送信計画に補う
    assert d.plan == {'action': 'goto', 'targets': ['south_aisle', 'shipping']} and d.source == 'llm'


def test_expansion_keeps_route_off_blocked_aisle():
    # 南通路を通行止めにしたとき、西ホール経由の区間は中央通路を通るように補われる
    p = Planner('warehouse', ScriptedLLM(goto('west_hall', 'shipping')))
    p.blocked = {'south_aisle'}
    d = p.handle_user('西ホールを経由して出荷口へ')
    assert 'south_aisle' not in d.plan['targets']
    assert d.plan['targets'][-1] == 'shipping'


def test_blocked_report_marks_aisle_and_retries_bad_plan():
    llm = ScriptedLLM(goto('shipping'),                      # 最初の指示
                      goto('shipping'),                      # 再計画1回目: 迂回なし → 検証NG
                      goto('middle_aisle', 'shipping'))      # 修正後
    p = Planner('warehouse', llm)
    p.handle_user('出荷口へ')
    d = p.handle_blocked(blocked_status('south_aisle', ['shipping'], [2.95, 1.05]))
    assert 'south_aisle' in p.blocked
    assert d.plan['targets'] == ['middle_aisle', 'shipping'] and d.source == 'llm'
    # 検証エラーの内容が LLM に返されている
    assert '検証エラー' in llm.calls[-1][-1]['content']


def test_fallback_when_llm_keeps_failing():
    p = Planner('warehouse', ScriptedLLM(goto('shipping')))
    p.handle_user('出荷口へ')
    d = p.handle_blocked(blocked_status('south_aisle', ['shipping'], [2.95, 1.05]))
    assert d.source == 'fallback' and d.plan['targets'] == ['middle_aisle', 'shipping']


def test_second_block_goes_north():
    p = Planner('warehouse', ScriptedLLM(goto('shipping')))
    p.handle_user('出荷口へ')
    p.handle_blocked(blocked_status('south_aisle', ['shipping'], [2.95, 1.05]))
    d = p.handle_blocked(blocked_status('middle_aisle', ['middle_aisle', 'shipping'], [2.4, 2.5]))
    assert d.plan['targets'] == ['north_aisle', 'shipping']


def test_all_blocked_gives_up():
    p = Planner('warehouse', ScriptedLLM(goto('shipping')))
    p.blocked = {'south_aisle', 'middle_aisle', 'north_aisle'}
    d = p.handle_user('出荷口へ')
    assert d.plan is None and d.source == 'none'


def test_user_report_updates_blocked():
    p = Planner('warehouse', ScriptedLLM(
        {'action': 'info', 'targets': [], 'blocked_add': ['south_aisle'], 'reply': '了解'}))
    d = p.handle_user('南通路は荷物で塞がってるよ')
    assert d.plan is None and p.blocked == {'south_aisle'}


def test_replan_limit():
    p = Planner('warehouse', ScriptedLLM(goto('shipping')), max_replans=1)
    p.handle_user('出荷口へ')
    p.handle_blocked(blocked_status('south_aisle', ['shipping'], [2.95, 1.05]))
    d = p.handle_blocked(blocked_status('middle_aisle', ['shipping'], [2.4, 2.5]))
    assert d.plan is None


def test_invalid_id_is_rejected_then_fixed():
    llm = ScriptedLLM({'action': 'goto', 'targets': ['moon'], 'reply': ''}, goto('charger'))
    p = Planner('warehouse', llm)
    d = p.handle_user('充電して')
    assert d.plan['targets'][-1] == 'charger'


# ---------- A: 最短性の検証 ----------
def test_longer_route_is_sent_back_when_shorter_exists():
    llm = ScriptedLLM(goto('north_aisle', 'shipping'), goto('shipping'))
    p = Planner('warehouse', llm)
    d = p.handle_user('出荷口へ行って')
    assert len(llm.calls) == 2 and '最短でない' in llm.calls[-1][-1]['content']
    assert d.plan['targets'] == ['south_aisle', 'shipping']


def test_shortest_among_unblocked_aisles_is_required():
    # 南が通行止めのとき、北(約10 m)は通行止めを避けていても中央(約7.5 m)より長いので差し戻す
    llm = ScriptedLLM(goto('north_aisle', 'shipping'), goto('middle_aisle', 'shipping'))
    p = Planner('warehouse', llm)
    p.blocked = {'south_aisle'}
    d = p.handle_user('出荷口へ行って')
    assert d.plan['targets'] == ['middle_aisle', 'shipping']
    assert any('検証NG: 経路が最短でない' in l for l in d.log)


def test_longest_route_is_accepted_when_it_is_the_only_one():
    p = Planner('warehouse', ScriptedLLM(goto('north_aisle', 'shipping')))
    p.blocked = {'south_aisle', 'middle_aisle'}
    d = p.handle_user('出荷口へ行って')
    assert d.plan['targets'] == ['north_aisle', 'shipping'] and len(d.log) == 2


def test_shortness_check_can_be_disabled():
    p = Planner('warehouse', ScriptedLLM(goto('north_aisle', 'shipping')), shortest_tol=None)
    d = p.handle_user('出荷口へ行って')
    assert d.plan['targets'] == ['north_aisle', 'shipping']


# ---------- B: 到着できた計画の再利用 ----------
def test_experience_saved_only_after_arrival_and_reused(tmp_path):
    from experience import Experience
    path = str(tmp_path / 'exp.json')
    p = Planner('warehouse', ScriptedLLM(goto('middle_aisle', 'charger')), experience=Experience(path))
    p.blocked = {'south_aisle'}
    p.handle_user('充電ステーションへ行って')
    assert len(p.experience) == 0                 # 到着前は保存しない
    p.on_status({'state': 'done'})
    assert len(p.experience) == 1

    llm = ScriptedLLM(goto('middle_aisle', 'shipping'))
    q = Planner('warehouse', llm, experience=Experience(path, read_only=True))
    q.blocked = {'south_aisle'}
    d = q.handle_user('出荷口へ行って')
    prompt = llm.calls[0][-1]['content']
    assert '過去に到着できた計画' in prompt and '充電ステーション' in prompt
    q.on_status({'state': 'done'})
    assert len(q.experience) == 1                 # 読み取り専用は増えない
    # 通行止めの状況が違えば参照しない
    r_llm = ScriptedLLM(goto('shipping'))
    r = Planner('warehouse', r_llm, experience=Experience(path, read_only=True))
    r.handle_user('出荷口へ行って')
    assert '過去に到着できた計画' not in r_llm.calls[0][-1]['content']


def test_longer_route_is_not_saved_as_experience(tmp_path):
    from experience import Experience
    p = Planner('warehouse', ScriptedLLM(goto('north_aisle', 'shipping')), shortest_tol=None,
                experience=Experience(str(tmp_path / 'e.json')))
    p.handle_user('出荷口へ行って')
    p.on_status({'state': 'done'})
    assert len(p.experience) == 0


# ---------- D: 口頭解除した通路の事前確認 ----------
def test_voice_cleared_aisle_is_marked_for_verification():
    p = Planner('warehouse', ScriptedLLM(goto('shipping')))
    p.blocked = {'south_aisle'}
    p.handle_user('南通路は片付いた')
    assert 'south_aisle' not in p.blocked and 'south_aisle' in p.cleared
    d = p.handle_user('出荷口へ行って')
    assert d.plan['targets'] == ['south_aisle', 'shipping']
    assert d.plan['verify'] == ['south_aisle']
    p.on_status({'state': 'arrived', 'target': 'south_aisle'})
    assert 'south_aisle' not in p.cleared             # 通過後は確認の対象から外す


def test_plan_without_cleared_aisle_has_no_verify_field():
    p = Planner('warehouse', ScriptedLLM(goto('shipping')))
    d = p.handle_user('出荷口へ行って')
    assert 'verify' not in d.plan


def test_precheck_obstacle_registers_blocked_and_replans():
    llm = ScriptedLLM(goto('shipping'), goto('middle_aisle', 'shipping'))
    p = Planner('warehouse', llm)
    p.blocked = {'south_aisle'}
    p.handle_user('南通路は片付いた')
    p.handle_user('出荷口へ行って')
    d = p.handle_blocked({'state': 'blocked', 'near_aisle': 'south_aisle', 'cause': 'obstacle',
                          'scan_hits': 9, 'target': 'south_aisle',
                          'remaining': ['south_aisle', 'shipping'], 'pose': [1.5, 1.0]})
    assert 'south_aisle' in p.blocked and 'south_aisle' not in p.cleared
    assert d.plan['targets'] == ['middle_aisle', 'shipping']


# ---------- E: 停止原因の分類 ----------
def test_unconfirmed_stall_is_not_registered_and_retried_once():
    p = Planner('warehouse', ScriptedLLM(goto('shipping')))
    p.handle_user('出荷口へ行って')
    st = {'state': 'blocked', 'near_aisle': 'middle_aisle', 'cause': 'unconfirmed', 'scan_hits': 0,
          'target': 'middle_aisle', 'remaining': ['middle_aisle', 'shipping'], 'pose': [2.6, 2.5]}
    d = p.handle_blocked(st)
    assert p.blocked == set() and d.source == 'retry'
    assert d.plan['targets'] == ['middle_aisle', 'shipping'] and '障害物を確認できません' in ''.join(d.log)
    d2 = p.handle_blocked(st)                           # 2 回目は指示を求めて停止
    assert d2.plan is None and p.blocked == set()


def test_obstacle_cause_registers_as_before():
    llm = ScriptedLLM(goto('shipping'), goto('middle_aisle', 'shipping'))
    p = Planner('warehouse', llm)
    p.handle_user('出荷口へ行って')
    d = p.handle_blocked({'state': 'blocked', 'near_aisle': 'south_aisle', 'cause': 'obstacle',
                          'scan_hits': 8, 'target': 'south_aisle',
                          'remaining': ['south_aisle', 'shipping'], 'pose': [1.5, 1.0]})
    assert p.blocked == {'south_aisle'} and d.plan['targets'] == ['middle_aisle', 'shipping']


def test_report_without_cause_is_treated_as_obstacle():
    llm = ScriptedLLM(goto('shipping'), goto('middle_aisle', 'shipping'))
    p = Planner('warehouse', llm)
    p.handle_user('出荷口へ行って')
    p.handle_blocked({'state': 'blocked', 'near_aisle': 'south_aisle', 'target': 'south_aisle',
                      'remaining': ['south_aisle', 'shipping'], 'pose': [1.5, 1.0]})
    assert p.blocked == {'south_aisle'}


# ---------- 通行止めの申告は規則で登録する ----------
def test_block_report_alone_registers_without_moving():
    llm = ScriptedLLM(goto('middle_aisle', 'shipping'))     # LLM が移動と誤解しても使われない
    p = Planner('warehouse', llm)
    d = p.handle_user('南通路が塞がっている')
    assert p.blocked == {'south_aisle'} and d.plan is None and d.source == 'block'
    assert llm.calls == []


def test_block_report_with_destination_registers_then_plans():
    llm = ScriptedLLM(goto('shipping'), goto('middle_aisle', 'shipping'))
    p = Planner('warehouse', llm)
    d = p.handle_user('南通路が塞がっているので出荷口へ行って')
    assert 'south_aisle' in p.blocked
    assert d.plan['targets'] == ['middle_aisle', 'shipping']


def test_block_then_clear_then_move_marks_verify():
    p = Planner('warehouse', ScriptedLLM(goto('shipping')))
    p.handle_user('南通路が塞がっている')
    p.handle_user('南通路は片付いた')
    d = p.handle_user('出荷口へ行って')
    assert p.blocked == set() and d.plan['verify'] == ['south_aisle']
