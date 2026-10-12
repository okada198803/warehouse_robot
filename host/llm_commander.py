"""日本語の指示をローカル LLM で計画に変換し、立ち往生したら自動で再計画する。

    python llm_commander.py                      # 倉庫ワールド、ROS に送信
    python llm_commander.py --world field        # 春ロボフィールド
    python llm_commander.py --dry-run            # ROS なしで LLM の判断だけ確認
    python llm_commander.py --model qwen2.5:3b   # 軽量モデル
    python llm_commander.py --cargo              # 荷物の搬送: 「あれを出荷口へ」+ 指差し(pointing_cam.py)

対話中のコマンド(デモ・検証用)
    /block <通路id>   立ち往生の報告を模擬する(--dry-run で再計画を試す)
    /state           通行止め・現在地を表示
    q                終了

AI: Ollama 上のローカル LLM(無料)。出力は JSON スキーマで制約し、地点 id と経路を検証してから送信する。
"""
import argparse
import json
import os
import queue
import threading
import time

import ollama

from experience import Experience
from planner import Planner
from pointing import clear_selection, read_selection

EXPERIENCE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'experience.json')


def make_llm(model):
    def llm_fn(messages, schema):
        resp = ollama.chat(model=model, messages=messages, format=schema,
                           options={'temperature': 0})
        return json.loads(resp['message']['content'])
    return llm_fn


def ts():
    return time.strftime('%H:%M:%S')


class Commander:

    def __init__(self, planner, link=None, verbose=True):
        self.planner = planner
        self.link = link
        self.verbose = verbose
        self.events = queue.Queue()
        threading.Thread(target=self.worker, daemon=True).start()
        if link:
            link.on_status(lambda st: self.events.put(('status', st)))

    def worker(self):
        """LLM 呼び出しを 1 本のスレッドで直列に処理する。"""
        while True:
            kind, payload = self.events.get()
            try:
                if kind == 'user':
                    text, pointed = payload
                    d = self.planner.handle_user(text, pointed=pointed)
                    self.show(d)
                    if pointed and d.plan and d.plan.get('targets', [None])[0] == pointed:
                        clear_selection()   # 指差しは 1 回の指示に使い切る
                elif kind == 'status':
                    self.on_status(payload)
            except ollama.ResponseError as e:
                print(f'[{ts()}] Ollama エラー: {e}')
            except Exception as e:   # 1 件の失敗で全体を止めない
                print(f'[{ts()}] エラー: {e!r}')
            print('> ', end='', flush=True)

    def on_status(self, st):
        self.planner.on_status(st)
        state = st.get('state')
        if state == 'navigating':
            return
        name = st.get('target_name', '')
        print(f'\n[{ts()}] status: {state} {name} {st.get("message", "")}'.rstrip())
        if state == 'blocked':   # 誤検知の確認用に、停止位置と判定した通路を表示する
            print(f'    pose={st.get("pose")} near_aisle={st.get("near_aisle")} remaining={st.get("remaining")}'
                  f' cause={st.get("cause")} scan_hits={st.get("scan_hits")}')
        if state == 'blocked':
            print(f'[{ts()}] 再計画します ...')
            self.show(self.planner.handle_blocked(st))

    def show(self, d):
        if self.verbose:
            for line in d.log:
                print(f'    {line}')
        print(f'[{ts()}] ロボット: {d.reply}')
        if d.plan:
            route = ' → '.join(self.planner.places[t]['name'] for t in d.plan.get('targets', []))
            tag = '(経路探索)' if d.source == 'fallback' else ''
            verify = '、'.join(self.planner.places[t]['name'] for t in d.plan.get('verify', []))
            print(f'[{ts()}] 送信{tag}: {d.plan["action"]} {route}' + (f'  (LiDAR で事前確認: {verify})' if verify else ''))
            if self.link:
                self.link.send_plan(d.plan)

    def simulate_block(self, aisle):
        """--dry-run 用: 通路の手前で立ち往生した報告を作る。"""
        p = self.planner.places[aisle]
        remaining = list(self.planner.active)
        st = {'state': 'blocked', 'near_aisle': aisle, 'pose': [p['x'] - 0.6, p['y']],
              'target': remaining[0] if remaining else None, 'remaining': remaining}
        self.events.put(('status', st))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='warehouse', choices=['warehouse', 'field'])
    ap.add_argument('--model', default='qwen2.5:7b')
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--dry-run', action='store_true', help='ROS に接続しない')
    ap.add_argument('--no-experience', action='store_true',
                    help='到着できた計画の記録(experience.json)を使わず、保存もしない')
    ap.add_argument('--cargo', action='store_true',
                    help='荷物の搬送に対応する(荷物置き場を使い、「あれ」を指差し(pointing_cam.py)で解決する)')
    ap.add_argument('--quiet', action='store_true', help='LLM 案と検証ログを表示しない')
    args = ap.parse_args()

    exp = None if args.no_experience else Experience(EXPERIENCE_FILE)
    planner = Planner(args.world, make_llm(args.model), experience=exp, cargo=args.cargo)
    if exp is not None:
        print(f'経験 {len(exp)} 件を参照(到着した最短の計画は追加で保存)')
    link = None
    if not args.dry_run:
        from ros_link import RosLink
        link = RosLink(args.host, args.port)
        print(f'connected to rosbridge {args.host}:{args.port}')
    cmd = Commander(planner, link, verbose=not args.quiet)

    print(f'model: {args.model} / world: {args.world}')
    print('地点: ' + '、'.join(p['name'] for p in planner.places.values()))
    print('指示を入力(例: 出荷口まで荷物を運んで)。/block <通路id> で立ち往生を模擬。終了は q')
    try:
        while True:
            text = input('> ').strip()
            if text in ('q', 'quit', 'exit'):
                break
            if not text:
                continue
            if text == '/state':
                print(f'通行止め: {sorted(planner.blocked)} / 現在地: {planner.pose or planner.last_arrived}')
            elif text.startswith('/block'):
                aisle = text.split()[-1]
                if aisle not in planner.aisles:
                    print(f'通路 id: {planner.aisles}')
                    continue
                cmd.simulate_block(aisle)
            else:
                pointed = read_selection() if args.cargo else None
                if pointed:
                    print(f'    指差し: {planner.places[pointed]["name"]}')
                cmd.events.put(('user', (text, pointed)))
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        if link:
            link.close()


if __name__ == '__main__':
    main()
