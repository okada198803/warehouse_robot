"""LLM による計画生成と、経路検証・再計画のループ(LLM 呼び出しは関数として注入)。

流れ
  1. 指示 or 立ち往生の報告を、地点一覧・通行止め・現在地と一緒に LLM へ渡す
  2. LLM は JSON(action, targets, reply, blocked_add, blocked_clear)を返す
  3. 地点 id の検証(plan_logic)と、静的地図上の経路検証(route_check)を行う
  4. 問題があれば理由を LLM に返して作り直させる(最大 max_retries 回)
  5. それでも通らなければ、経路探索による迂回案(suggest_detour)に切り替える
"""
import json
import os
import sys
from dataclasses import dataclass, field

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
AI_NAV_DIR = os.path.join(HERE, '..', 'ros2_ws', 'src', 'ai_nav')
sys.path.insert(0, AI_NAV_DIR)
from ai_nav.plan_logic import MAX_LOOPS, load_places, parse_plan  # noqa: E402
from ai_nav.route_check import RouteChecker  # noqa: E402

HISTORY_TURNS = 6
DEMONSTRATIVES = ('あれ', 'それ', 'これ', 'あの荷物', 'その荷物', 'この荷物')   # 指差しで荷物を指す言い方
CARGO_RULES = (
    '- 荷物の搬送(「〇〇の荷物を△△へ運んで」「あれを△△へ」)は、targets を [荷物置き場, △△] の順にする(荷物置き場 → 目的地)\n'
    '- 利用者の発言に [指差し: ...] があるとき、「あれ」「それ」「これ」はその荷物置き場を指す\n'
    '- どの荷物か分からない場合(指差しも名前もない)は action を info にし、reply で「どの荷物ですか」と聞き返す\n')
CLEAR_WORDS = ('片付', '通れる', '通行可', '解除', '撤去', '空いて', 'どかし', 'どけ', '開通',
               '障害物はなくなっ', '障害物がなくなっ', '荷物はなくなっ', '荷物がなくなっ')
NOT_CLEAR_WORDS = ('ていない', 'てない', 'ではない', 'じゃない', 'まだ',
                   '?', '？', '教えて', 'ですか', 'ますか', 'かな')   # 否定・未完了・質問は解除と扱わない
BLOCK_WORDS = ('塞が', 'ふさが', '通れない', '通行止め', '使えない', '通行不可', '置かれ', '詰ま')
CLEAR_HINT = ('- 解除の言い方の例: 「なくなった」「空いた」「どかした」「撤去した」「もう大丈夫」「問題ない」「元に戻った」。'
              'これらは blocked_clear に入れ、action は info にする\n'
              '- 「まだ」「〜ていない」などの否定・未完了、質問(「通れる?」)、状況を尋ねる発言は解除ではない。blocked_clear は空にする\n')
MAX_UNKNOWN_RETRIES = 1   # 通路を特定できない停止の再試行回数
SHORTEST_TOL = 0.10       # 最短経路より何割まで長くてよいか
SHORTEST_MARGIN = 0.3     # [m] 短い経路では、この距離までの差は許容する


def world_files(world):
    return (os.path.join(AI_NAV_DIR, 'config', f'places_{world}.yaml'),
            os.path.join(AI_NAV_DIR, 'maps', f'{world}.yaml'))


@dataclass
class Decision:
    plan: dict = None            # 送信する計画(None なら送信しない)
    reply: str = ''              # ロボットとしての返答
    source: str = 'llm'          # llm / fallback / none
    log: list = field(default_factory=list)


class Planner:

    def __init__(self, world, llm_fn, max_retries=2, max_replans=6,
                 shortest_tol=SHORTEST_TOL, experience=None, clear_rules=True, clear_hint=False, strict_schema=True, cargo=False):
        places_file, map_file = world_files(world)
        with open(places_file, encoding='utf-8') as f:
            self.notes = (yaml.safe_load(f) or {}).get('notes', '')
        self.cargo = cargo                 # True なら荷物置き場(kind: cargo)を使い、指差し・積載に対応する
        self.places = {k: v for k, v in load_places(places_file).items()
                       if cargo or v.get('kind') != 'cargo'}
        self.carrying = None               # 積載中の荷物 id(実行系の loaded / unloaded 報告で更新)
        self.checker = RouteChecker(map_file, self.places)
        self.aisles = [k for k, v in self.places.items() if v.get('kind') == 'aisle']
        self.llm_fn = llm_fn
        self.max_retries = max_retries
        self.max_replans = max_replans
        self.shortest_tol = shortest_tol   # None なら最短性を検証しない
        self.experience = experience       # Experience。None なら経験を使わない
        self.pending = None                # 到着したら経験として保存する計画
        self.clear_rules = clear_rules     # False なら解除の語の規則を使わず、LLM だけで解釈する(比較用)
        self.clear_hint = clear_hint       # True ならプロンプトに解除の言い方の例と否定の扱いを足す
        self.user_text = ''
        self.strict_schema = strict_schema # True(既定)なら blocked_add / blocked_clear を必須にする。省略されると解除の解釈が反映されないため
        self.cleared = set()               # 口頭で解除され、まだ通っていない通路(走行前に LiDAR で確認する)

        self.home = next(iter(self.places))
        self.blocked = set()
        self.pose = None
        self.last_arrived = None
        self.instruction = None
        self.replans = 0
        self.unknown_stalls = 0  # 通路を特定できない停止の回数
        self.active = []         # 送信済みで未到達の地点列
        self.history = []
        self.schema = self._schema()
        self.schema_report = self._schema(strict=False)   # ロボットの停止報告への返答用(解除は受け付けないため必須にしない)
        self.system = self._system_prompt()

    # ---------- プロンプト ----------
    def _schema(self, strict=None):
        strict = self.strict_schema if strict is None else strict
        aisle_enum = {'type': 'string', 'enum': self.aisles or ['none']}
        return {
            'type': 'object',
            'properties': {
                'action': {'type': 'string', 'enum': ['goto', 'patrol', 'stop', 'info', 'unknown']},
                'targets': {'type': 'array',
                            'items': {'type': 'string', 'enum': list(self.places)}},
                'loops': {'type': 'integer', 'minimum': 1, 'maximum': MAX_LOOPS},
                'blocked_add': {'type': 'array', 'items': aisle_enum},
                'blocked_clear': {'type': 'array', 'items': aisle_enum},
                'reply': {'type': 'string'},
            },
            'required': ['action', 'targets', 'reply'] + (['blocked_add', 'blocked_clear'] if strict else []),
        }

    def _system_prompt(self):
        lines = []
        for pid, p in self.places.items():
            alias = '、'.join(p.get('aliases', []))
            kind = {'aisle': '【通路】', 'cargo': '【荷物置き場】'}.get(p.get('kind'), '')
            lines.append(f'- {pid}: {kind}{p["name"]}(別名: {alias})… {p["description"]}')
        places = '\n'.join(lines)
        hint = CLEAR_HINT if self.clear_hint else ''
        cargo_rules = CARGO_RULES if self.cargo else ''
        return f"""あなたは倉庫で働く移動ロボットの運行管理AIです。利用者の日本語の指示や、ロボットからの状況報告を受け取り、移動計画の JSON を出力します。

# 地点(id: 名前 … 説明)
{places}

# 構造
{self.notes or '(特記事項なし)'}

# 出力ルール
- action
  - goto: targets の地点へ順に移動する
  - patrol: targets を順に巡回し loops 回繰り返す(「巡回」「見回り」など)
  - stop: 停止。targets は空
  - info: 通路の状況連絡など、移動を伴わない発言。targets は空
  - unknown: 一覧にない場所・ロボットにできない指示。targets は空
- targets には一覧の id だけを使う
- 移動(goto)は、今回の利用者の発言で移動先が明示された場合か、「戻って」の場合だけ使う。直前の指示を繰り返さない
- 「〇〇通路が塞がっている/使えない」は blocked_add に通路 id を入れる
- 「〇〇通路は片付いた/通れるようになった」は blocked_clear に通路 id を入れ、action は info にする。この場合は移動しない
{hint}{cargo_rules}- 通行止めの通路は、目的地にも経由地にも使わない。通行止めの通路を通らないと着けない場合は、使える別の通路を経由地として目的地の前に入れる
- 使える通路が複数あるときは、距離の短いものを選ぶ
- 目的地が名指しされている場合(例: 「出荷口に戻って」)は、その地点を目的地にする。「戻って」だけの場合は {self.home}
- reply にはロボットとしての短い日本語(40 文字以内)。迂回する場合はどの通路を使うかを含める
"""

    def _context(self):
        blocked = '、'.join(self.places[a]['name'] for a in sorted(self.blocked)) or 'なし'
        if self.pose:
            here = f'座標 ({self.pose[0]:.2f}, {self.pose[1]:.2f})'
            if self.last_arrived:
                here += f'、直前の到着地点 {self.places[self.last_arrived]["name"]}'
        else:
            here = self.places[self.last_arrived or self.home]['name']
        carrying = f' / 積載中: {self.places[self.carrying]["name"]}' if self.cargo and self.carrying else ''
        return f'[状況] 現在地: {here} / 通行止め: {blocked}{carrying}'

    def _start(self):
        return tuple(self.pose) if self.pose else (self.last_arrived or self.home)

    # ---------- 判断 ----------
    def handle_user(self, text, pointed=None):
        """pointed: 指差しで確定した荷物置き場 id(--cargo のとき。「あれ」などの指示語を解決する)。"""
        self.instruction = text
        self.user_text = text
        self.replans = 0
        self.unknown_stalls = 0
        cleared = self._clear_targets(text) if self.clear_rules else []
        if cleared and not self._names_area(text) and '戻' not in text:
            return self._clear_only(cleared, text)     # 解除だけの発言: 移動しない
        blocked = self._block_targets(text) if not cleared else []
        if blocked:
            self.blocked.update(blocked)               # 通行止めの申告は LLM に任せず、規則で登録する
            self.cleared.difference_update(blocked)
            if not self._names_area(text) and '戻' not in text:
                return self._block_only(blocked, text)  # 申告だけの発言: 移動しない
        self.blocked.difference_update(cleared)        # 解除と移動の両方: 先に解除してから計画
        self.cleared.update(cleared)
        pick, ask = self._resolve_cargo(text, pointed)
        if ask:
            return ask
        prompt = text
        if pick and pointed == pick:
            prompt += f'\n[指差し: {self.places[pick]["name"]}({pick})]'
        return self._decide(f'{prompt}\n{self._context()}', record=text, clear_ids=cleared,
                            named=self._named_area(text), cargo_pick=pick)

    def _resolve_cargo(self, text, pointed):
        """荷物置き場を決める。(荷物 id | None, 聞き返しの Decision | None)。

        名前で指定(「南の荷物」)があればそれを使う。指示語(「あれ」)なら指差しの結果を使う。
        指示語があるのに指差しがなければ、聞き返す。
        """
        if not self.cargo:
            return None, None
        named = [pid for pid, p in self.places.items() if p.get('kind') == 'cargo'
                 and any(n in text for n in [p['name'], *p.get('aliases', [])])]
        if len(named) == 1:
            return named[0], None
        if any(w in text for w in DEMONSTRATIVES):
            if pointed in self.places and self.places[pointed].get('kind') == 'cargo':
                return pointed, None
            reply = 'どの荷物ですか? 指差して教えてください'
            self._remember(text, {'action': 'info', 'targets': [], 'reply': reply})
            return None, Decision(None, reply, 'ask', ['指示語があるが指差しが確定していない: 聞き返し'])
        return None, None

    def _named_area(self, text):
        """通路以外の地点が一つだけ名指しされていれば、その id を返す。"""
        hits = [pid for pid, p in self.places.items()
                if p.get('kind') not in ('aisle', 'cargo')
                and any(n in text for n in [p['name'], *p.get('aliases', [])])]
        return hits[0] if len(hits) == 1 else None

    def _clear_targets(self, text):
        """解除を表す語と通路の名前・別名が両方あれば、その通路 id を返す。"""
        if not any(w in text for w in CLEAR_WORDS) or any(w in text for w in NOT_CLEAR_WORDS):
            return []
        return [a for a in self.aisles
                if any(n in text for n in [self.places[a]['name'], *self.places[a].get('aliases', [])])]

    def _block_targets(self, text):
        """通行止めを表す語と通路の名前・別名が両方あれば、その通路 id を返す。"""
        if not any(w in text for w in BLOCK_WORDS):
            return []
        return [a for a in self.aisles
                if any(n in text for n in [self.places[a]['name'], *self.places[a].get('aliases', [])])]

    def _block_only(self, blocked, text):
        names = '、'.join(self.places[a]['name'] for a in blocked)
        reply = f'{names}を通行止めに登録しました'
        self._remember(text, {'action': 'info', 'targets': [], 'reply': reply})
        return Decision(None, reply, 'block', [f'通行止め登録: {names}'])

    def _names_area(self, text):
        """通路以外の地点の名前・別名を含むか。"""
        return any(any(n in text for n in [p['name'], *p.get('aliases', [])])
                   for p in self.places.values() if p.get('kind') not in ('aisle', 'cargo'))

    def _clear_only(self, cleared, text):
        self.blocked.difference_update(cleared)
        self.cleared.update(cleared)
        names = '、'.join(self.places[a]['name'] for a in cleared)
        reply = f'{names}の通行止めを解除しました'
        self._remember(text, {'action': 'info', 'targets': [], 'reply': reply})
        return Decision(None, reply, 'clear', [f'通行止め解除: {names}'])

    def _retry_unidentified(self, target, remaining, why='通路を特定できません'):
        """通路を特定できない、または障害物を確認できない停止: 通行止めには登録せず、同じ経路で再試行する。"""
        name = self.places[target]['name'] if target in self.places else '目的地'
        self.unknown_stalls += 1
        if self.unknown_stalls > MAX_UNKNOWN_RETRIES or not remaining:
            reply = f'{name}へ向かう途中で動けなくなりました({why})。指示をください'
            return Decision(None, reply, 'none',
                            [f'原因未確認の停止: 再試行上限に到達({self.unknown_stalls}回目)'])
        names = '、'.join(self.places[t]['name'] for t in remaining)
        reply = f'{name}へ向かう途中で動けなくなりました。通行止めとは判断せず、同じ経路で再試行します'
        return Decision({'action': 'goto', 'targets': remaining}, reply, 'retry',
                        [f'原因未確認の停止({why}): 通行止めに登録せず再試行({names})'])

    def handle_blocked(self, status):
        """plan_executor からの blocked 報告を受けて再計画する。"""
        near = status.get('near_aisle')
        if status.get('pose'):
            self.pose = tuple(status['pose'])
        target = status.get('target')
        remaining = [t for t in status.get('remaining', []) if t in self.places]
        if near not in self.places:
            # 停止位置がどの通路にも近くない場合、障害物と断定せず通行止めに登録しない
            return self._retry_unidentified(target, remaining)
        if status.get('cause') == 'unconfirmed':
            # 通路の近くで止まったが LiDAR で障害物を確認できない(経路追従の失敗など)場合も登録しない
            return self._retry_unidentified(target, remaining, '通路に障害物を確認できません')
        self.blocked.add(near)
        self.cleared.discard(near)
        if self.replans >= self.max_replans:
            return Decision(None, f'{self.max_replans}回迂回しても到着できませんでした。指示をください', 'none')
        self.replans += 1
        where = self.places[near]['name'] if near in self.places else '経路上'
        remain = '、'.join(self.places[t]['name'] for t in remaining) or 'なし'
        report = (f'[ロボットからの報告] {self.places[target]["name"] if target else "目的地"}へ移動中、'
                  f'{where}で障害物により進めなくなり停止しました。未到達の地点: {remain}。'
                  f'元の指示: 「{self.instruction or "不明"}」。'
                  f'通行止めを避けて残りの地点へ向かう計画を作り直してください。\n{self._context()}')
        decision = self._decide(report, record=f'(報告){where}で立ち往生', fallback_goal=(
            remaining[-1] if remaining else None), from_report=True)
        decision.log.insert(0, f'通行止めに追加: {where}')
        return decision

    def on_status(self, st):
        if st.get('pose'):
            self.pose = tuple(st['pose'])
        if st.get('state') == 'loaded':
            self.carrying = st.get('target')
        elif st.get('state') == 'unloaded':
            self.carrying = None
        if st.get('state') == 'arrived':
            self.last_arrived = st.get('target')
            self.cleared.discard(self.last_arrived)
            if self.active and self.active[0] == self.last_arrived:
                self.active.pop(0)
        if st.get('state') == 'done':
            self.replans = 0
            self._commit_experience()

    def _decide(self, prompt, record, fallback_goal=None, clear_ids=(), named=None, from_report=False,
                cargo_pick=None):
        messages = [{'role': 'system', 'content': self.system}]
        messages += self.history[-HISTORY_TURNS * 2:]
        examples = self._experience_text()
        messages.append({'role': 'user', 'content': f'{prompt}\n{examples}' if examples else prompt})
        log = []
        if examples:
            log.append('過去の成功例を参照: ' + examples.replace('\n', ' / '))
        out = None
        for attempt in range(self.max_retries + 1):
            out = self.llm_fn(messages, self.schema_report if from_report else self.schema)
            log.append(f'LLM案{attempt + 1}: {json.dumps(out, ensure_ascii=False)}')
            self._apply_blocked(out, allow_clear=not from_report)
            if named and out.get('action') == 'goto' and out.get('targets') \
                    and out['targets'][-1] != named:
                # 利用者が名指しした地点を最終目的地にする(LLM が別の地点へ向かわせた場合の補正)
                before = out['targets'][-1]
                log.append(f'目的地を補正: {self.places.get(before, {}).get("name", before)} → {self.places[named]["name"]}')
                out['targets'][-1] = named
            if cargo_pick and out.get('action') in ('goto', 'info', 'unknown'):
                self._fix_cargo(out, cargo_pick, named, log)
            action = out.get('action')
            if action in ('stop', 'info', 'unknown'):
                self._remember(record, out)
                plan = {'action': 'stop'} if action == 'stop' else None
                if plan:
                    self.active = []
                return Decision(plan, out.get('reply', ''), 'llm', log)

            plan = {k: out[k] for k in ('action', 'targets', 'loops') if k in out}
            try:
                _, targets = parse_plan(json.dumps(plan), self.places)
                problems, length = self.checker.check_plan(self._start(), targets, self.blocked)
            except ValueError as e:
                problems, length = [str(e)], 0.0
            if not problems:
                problems = self._shortness_problems(plan, targets, length)
            if not problems:
                log.append(f'検証OK: 経路長 約{length:.1f} m')
                targets = self._expand(self._start(), targets)
                plan = {**plan, 'targets': targets}
                verify = [t for t in targets if t in self.cleared]
                if verify:
                    plan['verify'] = verify   # 口頭で解除された通路は、走行前に LiDAR で確認させる
                self.active = list(targets)
                self._remember(record, out)
                self._set_pending(plan, length)
                reply = self._route_reply(targets) if plan.get('action') == 'goto' else out.get('reply', '')
                return Decision(plan, reply, 'llm', log)
            log.append('検証NG: ' + ' / '.join(problems))
            hint = ''
            if clear_ids:
                names = '、'.join(self.places[a]['name'] for a in clear_ids)
                hint = f'\n利用者の発言は通行止めの解除({names})です。その通路 id を blocked_clear に入れてください。'
            messages.append({'role': 'assistant', 'content': json.dumps(out, ensure_ascii=False)})
            messages.append({'role': 'user', 'content':
                             '[検証エラー] ' + ' / '.join(problems) + hint + '\n修正した計画を JSON で出力してください。'})
            if fallback_goal is None and out.get('targets'):
                fallback_goal = out['targets'][-1]

        # LLM が検証を通る計画を出せなかった → 経路探索で迂回案を作る
        if fallback_goal and fallback_goal not in self.blocked:
            detour = self.checker.suggest_detour(self._start(), fallback_goal, self.blocked)
            if detour:
                via = [self.places[t]['name'] for t in detour[:-1]]
                reply = (f'{via[0]}経由で{self.places[fallback_goal]["name"]}へ向かいます'
                         if via else f'{self.places[fallback_goal]["name"]}へ向かいます')
                log.append(f'経路探索の迂回案を採用: {detour}')
                detour = self._expand(self._start(), detour)
                plan = {'action': 'goto', 'targets': detour}
                self.active = list(detour)
                self._remember(record, {**plan, 'reply': reply})
                return Decision(plan, reply, 'fallback', log)
        reply = '使える通路がなく到着できません。通路の確認をお願いします'
        self._remember(record, {'action': 'unknown', 'targets': [], 'reply': reply})
        return Decision(None, reply, 'none', log)

    def _fix_cargo(self, out, pick, dest, log):
        """荷物の搬送は [荷物置き場, 目的地] にそろえる(LLM が荷物置き場を落とす・別の荷物にする場合の補正)。"""
        targets = [t for t in (out.get('targets') or []) if self.places.get(t, {}).get('kind') != 'cargo']
        if dest and (not targets or targets[-1] != dest):
            targets.append(dest)
        fixed = [pick] + targets
        if out.get('action') != 'goto' or out.get('targets') != fixed:
            log.append('荷物の搬送に補正: ' + ' → '.join(self.places[t]['name'] for t in fixed))
        out['action'] = 'goto'
        out['targets'] = fixed

    def _best(self, targets):
        """最終地点までの最短の計画と距離。比較できない計画(途中に通路以外の地点がある等)は None。"""
        if not targets or len(targets) > 2 or self.places[targets[-1]].get('kind') == 'aisle':
            return None
        if any(self.places[t].get('kind') != 'aisle' for t in targets[:-1]):
            return None
        best, best_len = self.checker.best_plan(self._start(), targets[-1], self.blocked)
        return (best, best_len) if best else None

    def _shortness_problems(self, plan, targets, length):
        """検証の段階 3: 通行止めを避けていても、最短でなければ差し戻す。"""
        if self.shortest_tol is None or plan.get('action') != 'goto':
            return []
        found = self._best(targets)
        if not found:
            return []
        best, best_len = found
        if length <= best_len * (1 + self.shortest_tol) + SHORTEST_MARGIN:
            return []
        via = '、'.join(self.places[t]['name'] for t in best[:-1]) or '直接'
        return [f'経路が最短でない(約{length:.1f} m)。通行止め以外で最も短いのは'
                f'{via}経由(約{best_len:.1f} m)。距離の短い通路を選ぶこと']

    def _set_pending(self, plan, length):
        """到着(done)したら経験として保存する計画を覚えておく。最短でない計画は保存しない。"""
        self.pending = None
        if self.experience is None or plan.get('action') != 'goto':
            return
        if any(self.places[t].get('kind') == 'cargo' for t in plan['targets']):
            return   # 荷物の搬送は、通常の経験(通行止めの組合せ)に混ぜない
        found = self._best(plan['targets'])
        tol = self.shortest_tol if self.shortest_tol is not None else SHORTEST_TOL
        if found and length > found[1] * (1 + tol) + SHORTEST_MARGIN:
            return
        self.pending = {'blocked': set(self.blocked), 'instruction': self.instruction or '',
                        'targets': list(plan['targets']), 'length': length}

    def _commit_experience(self):
        if self.pending and self.experience is not None:
            self.experience.add(**self.pending)
        self.pending = None

    def _experience_text(self):
        if self.experience is None:
            return ''
        similar = bool(getattr(self.experience, 'similar', False))
        hits = self.experience.retrieve(self.blocked, similar=similar)
        if not hits:
            return ''
        if any(h.get('similar') for h in hits):
            lines = ['[過去に到着できた計画(参考。通行止めの状況は現在と異なる。そのまま使わず、現在の通行止めを避けること)]']
        else:
            lines = ['[過去に到着できた計画(参考。同じ通行止めの状況)]']
        for h in hits:
            route = ' → '.join(self.places[t]['name'] for t in h['targets'] if t in self.places)
            blocked = '、'.join(self.places[a]['name'] for a in h['blocked'] if a in self.places) or 'なし'
            lines.append(f'- 通行止め: {blocked} / 指示: 「{h["instruction"]}」 / 計画: {route}(約{h["length"]} m)')
        return '\n'.join(lines)

    def _route_reply(self, targets):
        """返答文は送信する経路から作る(LLM の経路説明と送信内容がずれないため)。"""
        names = [self.places[t]['name'] for t in targets]
        if self.places[targets[0]].get('kind') == 'cargo':
            if len(targets) == 1:
                return f'{names[0]}へ向かい、積載します'
            via = names[1:-1]
            way = f'{"、".join(via)}経由で' if via else ''
            return f'{names[0]}を積載し、{way}{names[-1]}へ運びます'
        via = [n for n in names[:-1]]
        if via:
            return f'{"、".join(via)}経由で{names[-1]}へ向かいます'
        return f'{names[-1]}へ向かいます'

    def _expand(self, start, targets):
        """各区間の検証済み経路が通る通路を補う。Nav2 が通行止めの通路を選ばないようにするため。"""
        out, prev = [], start
        for t in targets:
            path, _ = self.checker.shortest_path(prev, t, avoid=self.blocked)
            if path:
                for a in self.checker.aisles_in_order(path):
                    if a != t and (not out or out[-1] != a):
                        out.append(a)
            out.append(t)
            prev = t
        return out

    def _apply_blocked(self, out, allow_clear=True):
        for a in out.get('blocked_add', []) or []:
            if a in self.aisles:
                self.blocked.add(a)
                self.cleared.discard(a)
        negated = any(w in self.user_text for w in NOT_CLEAR_WORDS)
        for a in out.get('blocked_clear', []) or []:
            if negated or not allow_clear or a not in self.blocked:
                continue          # 否定・未完了の発言、ロボットからの報告への返答では解除しない
            self.blocked.discard(a)
            self.cleared.add(a)   # LLM が解除と解釈した通路も、走行前に LiDAR で確認させる

    def _remember(self, user_text, out):
        self.history += [{'role': 'user', 'content': user_text},
                         {'role': 'assistant', 'content': json.dumps(out, ensure_ascii=False)}]
