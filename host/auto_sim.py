"""Gazebo での検証 V2 / V3 / V5(V6 を含む)を無人で繰り返し実行する。

    ./run_auto.sh v2 v3 v5          # 通常はこのラッパー経由で実行(異常時にシミュレータを再起動する)
    python auto_sim.py --suite v3 --n 10

前提: Docker のコンテナ(pai-ros)が起動済み、シミュレータ一式が起動済み(run_auto.sh が起動する)、
      Ollama が起動済み(V2 はスタブ LLM のため不要)。

試行ごとの流れ: ソフトリセット(停止・パレット撤去・ロボットを初期位置へ移動・コストマップ消去)
  → 初期状態の設定(パレット設置・事前確認の ON/OFF)→ 指示 → blocked/再計画を繰り返して到着まで
  → 記録(CSV 1 行 + 経過ログ JSONL)。結果は ../docs/auto/<suite>.csv。再実行すると完了済みの試行は飛ばす。

終了コード: 0 完了 / 75 シミュレータの再起動が必要(run_auto.sh が再起動して続きから再開) / その他 異常
"""
import argparse
import csv
import json
import os
import queue
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..'))
OUT = os.path.join(ROOT, 'docs', 'auto')
LOGS = os.path.join(OUT, 'logs')
DOCKER_DIR = os.path.join(ROOT, 'docker')
SRC = 'source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && '
START = (0.8, 0.8)
TRIAL_TIMEOUT = 300.0      # 1 試行の上限 [s]
RESTART = 75

FIELDS = ['suite', 'cond', 'trial', 'ok', 'error', 't_first_blocked', 't_total', 'n_blocked', 'blocks',
          'final_blocked', 'false_register', 'fallback_used', 'first_plan', 'plans', 'llm_calls',
          'detected_by', 'started_at']


class NeedRestart(Exception):
    pass


def sh(cmd, timeout=60, check=False):
    """コンテナ内で ROS 2 のコマンドを実行する。(終了コード, 出力)"""
    try:
        p = subprocess.run(['docker', 'compose', 'exec', '-T', '-u', 'ubuntu', 'ros', 'bash', '-c', SRC + cmd],
                           cwd=DOCKER_DIR, stdin=subprocess.DEVNULL, capture_output=True, text=True,
                           timeout=timeout)
    except subprocess.TimeoutExpired:
        if check:
            raise
        return 124, 'timeout'
    out = (p.stdout or '') + (p.stderr or '')
    if check and p.returncode != 0:
        raise RuntimeError(f'{cmd!r} -> {p.returncode}: {out[-300:]}')
    return p.returncode, out


# ---------- シミュレータの状態 ----------
def healthy():
    rc, out = sh('ros2 lifecycle get /bt_navigator && ros2 node list | grep -q plan_executor && echo OK', timeout=40)
    return rc == 0 and 'active' in out and 'OK' in out


def robot_xy():
    rc, out = sh('timeout 10 ros2 topic echo --once --field pose.pose.position /odom', timeout=30)
    m = re.search(r'x:\s*(-?[\d.eE+-]+)\s*\n\s*y:\s*(-?[\d.eE+-]+)', out)
    return (float(m.group(1)), float(m.group(2))) if m else None


def soft_reset(link):
    """停止 → パレット撤去 → 初期位置へ移動 → コストマップ消去。失敗したら再起動を要求する。"""
    link.send_plan({'action': 'stop'})
    time.sleep(2.0)
    sh('ros2 run ai_nav obstacle clear', timeout=60)
    req = 'name: \\"LidarRobo4\\" position {x: %.2f y: %.2f z: 0.075} orientation {w: 1.0}' % START
    for _ in range(2):
        sh('ign service -s /world/warehouse/set_pose --reqtype ignition.msgs.Pose '
           f'--reptype ignition.msgs.Boolean --timeout 3000 --req "{req}"', timeout=30)
    sh('ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"; '
       'ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"',
       timeout=40)
    time.sleep(2.0)
    xy = robot_xy()
    if xy is None or abs(xy[0] - START[0]) > 0.3 or abs(xy[1] - START[1]) > 0.3:
        raise NeedRestart(f'初期位置に戻せない: {xy}')


def place_pallet(aisle):
    sh(f'ros2 run ai_nav obstacle place {aisle}', timeout=60, check=True)
    time.sleep(1.5)


def set_param(name, value):
    rc, out = sh(f'ros2 param set /plan_executor {name} {value}', timeout=30)
    if rc != 0:
        raise NeedRestart(f'{name} を設定できない: {out[-200:]}')


def set_precheck(on):
    rc, out = sh(f'ros2 param set /plan_executor precheck {"true" if on else "false"}', timeout=30)
    if rc != 0:
        raise NeedRestart(f'precheck を設定できない: {out[-200:]}')


# ---------- 試行 ----------
def stub_llm(messages, schema):
    """V2 用: 常に通行止めの南通路を通る計画を返す(検証 NG → 差し戻し → フォールバックを起こす)。"""
    return {'action': 'goto', 'targets': ['south_aisle', 'shipping'], 'blocked_add': [],
            'blocked_clear': [], 'reply': '南通路経由で出荷口へ向かいます'}


class Run:
    """1 回の試行。ロボットへの送信、status の受信、記録をまとめる。"""

    def __init__(self, link, planner, logpath):
        self.link = link
        self.planner = planner
        self.q = queue.Queue()
        self.t0 = time.monotonic()
        self.events = []
        self.traj = []
        self.f = open(logpath, 'w', encoding='utf-8')
        self.t_plan = None
        self.blocks = []
        self.plans = []
        self.fallback = False
        self.llm_calls = 0

    def log(self, kind, **kw):
        rec = {'t': round(time.monotonic() - self.t0, 2), 'kind': kind, **kw}
        self.f.write(json.dumps(rec, ensure_ascii=False) + '\n')
        self.f.flush()

    def on_status(self, st):
        self.q.put((time.monotonic(), st))

    def on_odom(self, msg):
        p = msg['pose']['pose']['position']
        self.log('odom', x=round(p['x'], 3), y=round(p['y'], 3))

    def send(self, d, label):
        for line in d.log:
            self.log('planner', line=line)
        self.log('decision', label=label, source=d.source, reply=d.reply, plan=d.plan)
        if d.source == 'fallback':
            self.fallback = True
        if d.plan:
            self.plans.append((d.plan.get('targets') or []) if d.plan.get('action') == 'goto' else [])
            self.link.send_plan(d.plan)
            return True
        return False

    def wait_done(self, deadline):
        """done になるまで status を処理する。blocked は再計画して送り直す。結果: ok / fail:<理由>。"""
        while time.monotonic() < deadline:
            try:
                t, st = self.q.get(timeout=1.0)
            except queue.Empty:
                continue
            state = st.get('state')
            self.planner.on_status(st)
            if state != 'navigating':
                self.log('status', **st)
            if state == 'done':
                return 'ok', t
            if state in ('failed', 'rejected'):
                return f'fail:{state}', t
            if state == 'blocked':
                self.blocks.append({'t': round(t - self.t_plan, 1), 'near': st.get('near_aisle'),
                                    'cause': st.get('cause'), 'hits': st.get('scan_hits'),
                                    'msg': st.get('message', '')})
                d = self.planner.handle_blocked(st)
                if not self.send(d, 'replan'):
                    return 'fail:no_replan', t
        return 'fail:timeout', time.monotonic()


class Hub:
    """status と odom を、実行中の試行に振り分ける(購読は接続ごとに 1 回だけ)。"""

    def __init__(self, link):
        import roslibpy
        self.run = None
        link.on_status(lambda st: self.run and self.run.on_status(st))
        self.odom = roslibpy.Topic(link.ros, '/odom', 'nav_msgs/msg/Odometry', throttle_rate=200, queue_length=1)
        self.odom.subscribe(lambda m: self.run and self.run.on_odom(m))


def drain(q):
    while True:
        try:
            q.get_nowait()
        except queue.Empty:
            return


def run_trial(link, hub, spec, trial, args):
    from llm_commander import make_llm
    from planner import Planner

    tag = f'{spec["suite"]}_{spec["cond"]}_{trial:02d}'
    os.makedirs(LOGS, exist_ok=True)
    row = {'suite': spec['suite'], 'cond': spec['cond'], 'trial': trial, 'ok': False, 'error': '',
           'started_at': time.strftime('%Y-%m-%d %H:%M:%S')}
    soft_reset(link)
    if spec.get('precheck') is not None:
        set_precheck(spec['precheck'])
    set_param('scan_min_hits', spec.get('scan_min_hits', 3))   # 毎回設定し直す(E で 999 にするため)
    for a in spec.get('pallets', []):
        place_pallet(a)

    base = stub_llm if spec.get('stub') else make_llm(args.model)
    calls = [0]

    def llm_fn(messages, schema):
        calls[0] += 1
        return base(messages, schema)

    planner = Planner('warehouse', llm_fn)
    run = Run(link, planner, os.path.join(LOGS, tag + '.jsonl'))
    hub.run = run
    try:
        drain(run.q)
        result, t_end = 'ok', None
        for i, text in enumerate(spec['script']):
            run.log('user', text=text)
            d = planner.handle_user(text)
            run.t_plan = time.monotonic()
            if not run.send(d, 'user'):
                continue            # 申告・解除だけの発言: 移動しない
            result, t_end = run.wait_done(run.t_plan + TRIAL_TIMEOUT)
            if result != 'ok':
                break
        else:
            if run.t_plan is None:
                result = 'fail:no_plan'
        if spec.get('expect_stop'):
            # E: 原因を確認できない停止。登録せず 1 回再試行し、2 回目は指示を求めて停止するのが期待動作
            row['ok'] = (result == 'fail:no_replan' and len(run.blocks) == 2 and not planner.blocked)
            row['error'] = '' if row['ok'] else (result if result != 'fail:no_replan' else 'unexpected_blocks')
        else:
            row['ok'] = result == 'ok'
            row['error'] = '' if result == 'ok' else result
        row['t_total'] = round(t_end - run.t_plan, 1) if (t_end and run.t_plan) else ''
        row['n_blocked'] = len(run.blocks)
        row['t_first_blocked'] = run.blocks[0]['t'] if run.blocks else ''
        row['blocks'] = json.dumps(run.blocks, ensure_ascii=False)
        row['final_blocked'] = '|'.join(sorted(planner.blocked))
        allowed = set(spec.get('expected_blocked', []))
        row['false_register'] = bool(set(planner.blocked) - allowed) or any(
            b['near'] not in allowed for b in run.blocks if b['near'])
        row['fallback_used'] = run.fallback
        row['first_plan'] = '→'.join(run.plans[0]) if run.plans else ''
        row['plans'] = ' / '.join('→'.join(p) for p in run.plans)
        row['llm_calls'] = calls[0]
        row['detected_by'] = (('precheck' if run.blocks and 'precheck' in run.blocks[0]['msg'] else 'stall')
                              if run.blocks else '')
    finally:
        hub.run = None
        run.f.close()
        link.send_plan({'action': 'stop'})
    return row


# ---------- 試行の一覧 ----------
VERBAL = ['南通路は塞がれている', '南通路は片付いた。出荷口へ行って']


def trials(suite, n):
    if suite == 'v2':
        return [(dict(suite='v2', cond='fallback', stub=True, pallets=['south_aisle'],
                      script=['南通路は塞がれている', '出荷口へ行って'], expected_blocked=['south_aisle']), i)
                for i in range(1, (n or 3) + 1)]
    if suite == 'v3':
        out = []
        for i in range(1, (n or 10) + 1):      # 条件を交互に実行して時間経過の偏りを避ける
            out.append((dict(suite='v3', cond='D_precheck_on', precheck=True, pallets=['south_aisle'],
                             script=VERBAL, expected_blocked=['south_aisle']), i))
            out.append((dict(suite='v3', cond='D_precheck_off', precheck=False, pallets=['south_aisle'],
                             script=VERBAL, expected_blocked=['south_aisle']), i))
            out.append((dict(suite='v3', cond='D_control_cleared', precheck=True, pallets=[],
                             script=VERBAL, expected_blocked=[]), i))
            out.append((dict(suite='v3', cond='E_unconfirmed', precheck=False, pallets=['south_aisle'],
                             scan_min_hits=999, expect_stop=True, script=['出荷口へ行って'],
                             expected_blocked=[]), i))
        return out
    if suite == 'v5':    # 南通路のパレットで停止 → 中央通路へ迂回(V5a 中央誤検知の再発頻度と V6 を兼ねる)
        return [(dict(suite='v5', cond='south_pallet', precheck=True, pallets=['south_aisle'],
                      script=['出荷口へ行って'], expected_blocked=['south_aisle']), i)
                for i in range(1, (n or 20) + 1)]
    raise SystemExit(f'unknown suite {suite}')


def load_done(path):
    if not os.path.exists(path):
        return set()
    with open(path, encoding='utf-8-sig') as f:
        return {(r['cond'], int(r['trial'])) for r in csv.DictReader(f)}


def append_row(path, row):
    new = not os.path.exists(path)
    with open(path, 'a', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction='ignore')
        if new:
            w.writeheader()
        w.writerow({k: row.get(k, '') for k in FIELDS})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--suite', required=True, choices=['v2', 'v3', 'v5'])
    ap.add_argument('--n', type=int, default=None, help='回数(v2 既定 3 / v3 既定 10 / v5 既定 20)')
    ap.add_argument('--model', default='qwen2.5:7b')
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--max-errors', type=int, default=3, help='連続でこの回数失敗したら再起動を要求する')
    args = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f'{args.suite}.csv')
    todo = [(s, i) for s, i in trials(args.suite, args.n) if (s['cond'], i) not in load_done(path)]
    print(f'[{args.suite}] 残り {len(todo)} 試行', flush=True)
    if not todo:
        return 0

    if not healthy():
        print('シミュレータが応答しない → 再起動を要求(終了コード 75)')
        return RESTART
    if not args.suite == 'v2':
        # Ollama の初回遅延を避けるため、先に 1 回呼んで温める
        from llm_commander import make_llm
        make_llm(args.model)([{'role': 'user', 'content': '{}'}], {'type': 'object'})

    from ros_link import RosLink
    link = RosLink(args.host, 9090)
    hub = Hub(link)
    consecutive = 0
    try:
        for spec, i in todo:
            print(f'[{time.strftime("%H:%M:%S")}] {spec["suite"]} {spec["cond"]} #{i} ...', flush=True)
            try:
                row = run_trial(link, hub, spec, i, args)
            except NeedRestart as e:
                print(f'  再起動が必要: {e}', flush=True)
                return RESTART
            except Exception as e:
                print(f'  異常: {e!r}', flush=True)
                consecutive += 1
                if consecutive >= args.max_errors:
                    return RESTART
                continue
            append_row(path, row)
            print(f'  ok={row["ok"]} t={row.get("t_total")} blocked={row.get("n_blocked")} '
                  f'false_register={row.get("false_register")} {row["error"]}', flush=True)
            consecutive = 0 if row['ok'] else consecutive + 1
            if consecutive >= args.max_errors:
                print('  連続失敗 → シミュレータを再起動')
                return RESTART
    finally:
        try:
            link.close()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    os._exit(code)    # roslibpy の reactor スレッドが残るため強制終了する
