"""複数ロボットの協調。通路の取り合い(同じ通路を逆向きに使う計画)の検出と解決。

C3 ロボット間通信: 各ロボットが自分の意図(使う通路と向き)を /fleet/intent に出し、
    優先順位の高い相手と取り合うなら、自分が別の通路を選ぶ。全体を見る役はいない。
C1 統制 LLM: 全ロボットの状況を 1 か所に集め、LLM が台ごとに「避ける通路」を割り当てる。
    LLM の出力は検証し、通らなければ差し戻す(最大 2 回)。それでも通らなければ協調なし(C0)で動かす。
どちらも「避ける通路」を台ごとに決めるだけで、通行止めの記憶(planner.blocked)には登録しない。
"""
import json
import threading
import time

PRIORITY = ('r1', 'r2', 'r3')       # 前にいるほど優先


def _xy(planner, place):
    if isinstance(place, str):
        p = planner.places[place]
        return p['x'], p['y']
    return place


def intent_of(planner, ns, dest, avoid=()):
    """このロボットが dest へ向かうときに使う通路と向き。avoid の通路は通れないものとして経路を作る。"""
    start = planner._start()
    saved = planner.blocked
    planner.blocked = set(saved) | set(avoid)
    try:
        seq = planner._expand(start, [dest])
    finally:
        planner.blocked = saved
    aisle = next((t for t in seq[:-1] if planner.places[t].get('kind') == 'aisle'), None)
    direction = 'E' if planner.places[dest]['x'] > _xy(planner, start)[0] else 'W'
    return {'from': ns, 'aisle': aisle, 'dir': direction}


def conflict(a, b):
    """同じ通路を逆向きに使う(1 台しか通れない通路で行き違えない)。"""
    return bool(a['aisle'] and a['aisle'] == b['aisle'] and a['dir'] != b['dir'])


def resolve(planner, ns, dest, higher):
    """優先順位の高い相手の意図 higher と取り合わない通路を選ぶ。(自分の意図, 避ける通路の集合, 解決できたか)。"""
    avoid = set()
    for _ in range(len(planner.aisles) + 1):
        it = intent_of(planner, ns, dest, avoid)
        if not any(conflict(it, h) for h in higher):
            return it, avoid, True
        if it['aisle'] is None:
            break
        avoid.add(it['aisle'])
    return intent_of(planner, ns, dest), set(), False


# ---------- C3: ロボット間通信 ----------
class PeerChannel:
    """ロボット 1 台ぶんの通信口。ROS のトピック /fleet/intent で意図を送り合う(rosbridge 経由)。"""

    def __init__(self, ros, ns):
        import roslibpy
        self.ns, self.tag, self.seen = ns, '', {}
        self.topic = roslibpy.Topic(ros, '/fleet/intent', 'std_msgs/msg/String')
        self.topic.advertise()
        self.topic.subscribe(self._on)

    def _on(self, msg):
        try:
            d = json.loads(msg['data'])
        except ValueError:
            return
        if d.get('from') != self.ns and d.get('tag') == self.tag:
            self.seen[d['from']] = d

    def reset(self, tag, drop_p=0.0, delay=0.0, seed=None):
        """試行の開始。drop_p: 送信 1 回(4 回の繰り返しを含む)ごとに不達にする確率、delay: 送信までの遅れ [s]。
        通信の欠落・遅延の試験(C3x・C3h・C3d・C4x)用。既定値では従来と同じ動作。"""
        import random
        self.tag, self.seen = tag, {}
        self.drop_p, self.delay = drop_p, delay
        self.rng = random.Random(seed)
        self.dropped = 0

    def announce(self, intent, repeat=4, interval=0.4):
        """取りこぼしに備えて数回送る(別スレッド)。"""
        import roslibpy
        body = json.dumps({**intent, 'tag': self.tag})
        if getattr(self, 'drop_p', 0.0) and self.rng.random() < self.drop_p:
            self.dropped += 1          # 不達(この送信は相手に届かない)
            return
        delay = getattr(self, 'delay', 0.0)

        def send():
            if delay:
                time.sleep(delay)
            for _ in range(repeat):
                self.topic.publish(roslibpy.Message({'data': body}))
                time.sleep(interval)
        threading.Thread(target=send, daemon=True).start()

    def wait_for(self, names, timeout):
        """names の意図が届くまで待つ(届かなければ timeout で打ち切り、届いた分だけ返す)。"""
        end = time.monotonic() + timeout
        while time.monotonic() < end and not all(n in self.seen for n in names):
            time.sleep(0.05)
        return {n: self.seen[n] for n in names if n in self.seen}


# ---------- C1: 統制 LLM ----------
# 第 1 版は「避ける通路」を台ごとに出力させたが、検証に通る出力が 30 回中 0 回だった(docs/multi/C1a.csv)。
# 第 2 版は「各ロボットが通る通路」を 1 つずつ出力させ、避ける通路(それ以外)はコードで導く。
def central_schema(names, aisles):
    return {'type': 'object', 'required': ['route', 'reason'], 'additionalProperties': False,
            'properties': {
                'route': {'type': 'object', 'required': list(names), 'additionalProperties': False,
                          'properties': {n: {'type': 'string', 'enum': list(aisles)} for n in names}},
                'reason': {'type': 'string'}}}


def central_prompt(planners, dests, notes_extra=''):
    names = list(planners)
    lines = ['複数のロボットが倉庫内を同時に移動する。西ホールと東ホールは 南通路(幅0.9m)・中央通路(幅0.8m)・北通路(幅0.9m)'
             'の3本だけでつながり、どの通路も1台しか通れない。逆向きに進む2台が同じ通路に入ると行き違えず立ち往生する。',
             '各ロボットの現在地・目的地・通路ごとの距離を見て、立ち往生しないように、各ロボットが通る通路を1つずつ決めよ。',
             '逆向きのロボット同士は別の通路にする。同じ向きなら同じ通路でもよい。遠回りは時間が増えるので、'
             '全員が最短の通路で衝突しないならそのままにする。', '']
    for n in names:
        p = planners[n]
        start = p._start()
        sx = _xy(p, start)[0]
        home = start if isinstance(start, str) else 'unknown'
        d = dests[n]
        direction = '東向き' if p.places[d]['x'] > sx else '西向き'
        dist = []
        for a in p.aisles:
            problems, length = p.checker.check_plan(start, [a, d], set())
            if not problems:
                dist.append(f'{p.places[a]["name"]}({a}) {length:.1f}m')
        lines.append(f'{n}: 現在地 {p.places[home]["name"] if home in p.places else "座標(%.1f, %.1f)" % tuple(_xy(p, start))} → 目的地 {p.places[d]["name"]}'
                     f'({direction})。経由する通路ごとの距離: {" / ".join(dist)}')
    return '\n'.join(lines) + notes_extra


def check_assignment(planners, dests, avoid):
    """割り当て avoid(台 → 避ける通路の集合)を適用した計画に、通路の取り合いが残っていないか。問題点の一覧を返す。"""
    intents = {n: intent_of(planners[n], n, dests[n], avoid.get(n, ())) for n in planners}
    problems = []
    names = list(planners)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if conflict(intents[a], intents[b]):
                problems.append(f'{a} と {b} が{planners[a].places[intents[a]["aisle"]]["name"]}を逆向きに使う')
    return problems, intents


def central_assign(llm_fn, planners, dests, max_retries=2, occupied=None):
    """統制 LLM に各ロボットの通路を割り当てさせる。(avoid, 情報 dict)。検証に通らなければ avoid は空(協調なし)。
    occupied: {通路: 応答のないロボット名}(C4 用)。プロンプトに伝え、その通路の割り当ては検証で差し戻す。None なら従来どおり。"""
    names = list(planners)
    aisles = list(planners[names[0]].aisles)
    schema = central_schema(names, aisles)
    extra = ''
    if occupied:
        pl = planners[names[0]]
        extra = '\n' + '\n'.join(f'応答のないロボット {r} が{pl.places[a]["name"]}({a})の中で停止している。この通路は通れない。'
                                   for a, r in occupied.items())
    messages = [{'role': 'system', 'content': '倉庫の複数ロボットを統制する。出力は JSON のみ。'},
                {'role': 'user', 'content': central_prompt(planners, dests, extra)}]
    info = {'calls': 0, 'valid': False, 'reason': '', 'problems': [], 'raw': []}
    for _ in range(max_retries + 1):
        info['calls'] += 1
        try:
            out = llm_fn(messages, schema)
            info['raw'].append(out)
            route = out.get('route', {})
            # 割り当てた通路以外を「避ける」とする(割り当てが不正な台は避ける通路なし)
            avoid = {n: (set(aisles) - {route[n]}) if route.get(n) in aisles else set() for n in names}
            info['reason'] = out.get('reason', '')
        except Exception as e:      # LLM の応答不良は、検証失敗と同じ扱いにする
            info['problems'] = [f'LLM 応答エラー: {e!r}']
            continue
        problems, intents = check_assignment(planners, dests, avoid)
        for n in names:
            if route.get(n) not in aisles:
                problems.append(f'{n} の通路が不正')
            elif intents[n]['aisle'] != route[n]:
                problems.append(f'{n} は{route[n]}を通れない')
            elif occupied and route[n] in occupied:
                problems.append(f'{n} の{route[n]}には停止したロボット {occupied[route[n]]} がいる')
        info['problems'] = problems
        if not problems:
            info['valid'] = True
            return avoid, info
        messages.append({'role': 'assistant', 'content': json.dumps(out, ensure_ascii=False)})
        messages.append({'role': 'user', 'content': '問題があります: ' + '; '.join(problems) + '。作り直してください。'})
    return {n: set() for n in names}, info


def aisle_at(planner, xy, half_len=1.6):
    """座標 xy が入っている通路(なければ None)。通路の中心から x 方向 ±half_len、y 方向 ±0.45 m の範囲で判定する。"""
    for a in planner.aisles:
        p = planner.places[a]
        if abs(xy[0] - p['x']) <= half_len and abs(xy[1] - p['y']) <= 0.45:
            return a
    return None
