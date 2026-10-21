"""認識層(M8)の走行検証 P1: 口頭で「片付いた」と解除された通路に、停止ロボット(小さい障害物)が残っている場面。
R3(回帰の確認):単一の迂回(V5)と LiDAR の事前確認(V3)を、学習した認識器で再実行する。

    python auto_percep.py --n 10            # 通常は run_percep.sh 経由(シミュレータの起動・再起動を行う)
    python auto_percep.py --suite r3        # R3(SUITE=r3 bash run_percep.sh)
    python auto_percep.py --suite p1v3      # P1 を、通路幅で正規化した認識器(R1c の L2n、recognizer_v3.json)で(SUITE=p1v3 bash run_percep.sh)

条件(各 n 回、交互に実行):
  P_rule / P_offset / P_learned  南通路に停止ロボットを模した箱を置き、「南通路は塞がれている」→「南通路は片付いた。出荷口へ行って」
  C_rule / C_offset / C_learned  対照。箱を置かない(本当に片付いている)。事前確認の誤判定がないか
recognizer は plan_executor_v2 のパラメータ(rule = 元の規則、rule_offset = LiDAR の位置を補正した規則、learned = 学習した分類器)。
事前の予想(docs/perception/検証計画.md 4 章・R1 の結果から): P_rule・P_offset は事前確認で見逃して走り出し、立ち往生で検知。
P_learned は事前確認で検知する。C_* はいずれも誤判定なし。
凍結した auto_sim.py は変更せず、読み込んで使う(出力先と障害物だけを差し替える)。
"""
import argparse
import os
import sys
import time

import auto_sim as A

ROOT = A.ROOT
A.OUT = os.path.join(ROOT, 'docs', 'perception', 'runs')
A.LOGS = os.path.join(A.OUT, 'logs')
BOX = 'p1_robot_box'
BOX_SIZE = (0.30, 0.24, 0.20)      # 停止ロボットを模した箱(学習データの 'robot' と同じ大きさ)
BOX_POS = (3.5, 1.05)              # 南通路の中心(パレットと同じ位置)

_orig_reset = A.soft_reset


def remove_box():
    A.sh(f'ign service -s /world/warehouse/remove --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean '
         f'--timeout 3000 --req \'name: "{BOX}" type: MODEL\'', timeout=30)


def soft_reset(link, cargo=False):
    remove_box()
    _orig_reset(link, cargo)


def place_box(_aisle):
    sx, sy, sz = BOX_SIZE
    sdf = (f'<sdf version="1.6"><model name="{BOX}"><static>true</static><link name="l">'
           f'<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>'
           f'<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
           f'<material><ambient>0.9 0.5 0.1 1</ambient><diffuse>0.9 0.5 0.1 1</diffuse></material></visual>'
           f'</link></model></sdf>')
    A.sh(f"echo '{sdf}' > /tmp/{BOX}.sdf && ros2 run ros_ign_gazebo create -world warehouse -name {BOX} "
         f"-file /tmp/{BOX}.sdf -x {BOX_POS[0]} -y {BOX_POS[1]} -z {sz / 2}", timeout=60, check=True)
    time.sleep(1.5)


_orig_place = A.place_pallet
A.soft_reset = soft_reset
A.place_pallet = place_box


def trials(n):
    out = []
    for i in range(1, n + 1):
        for rec in ('rule', 'rule_offset', 'learned'):
            short = {'rule': 'rule', 'rule_offset': 'offset', 'learned': 'learned'}[rec]
            out.append((dict(suite='p1', cond=f'P_{short}', recognizer=rec, precheck=True, pallets=['south_aisle'],
                             script=A.VERBAL, expected_blocked=['south_aisle']), i))
            out.append((dict(suite='p1', cond=f'C_{short}', recognizer=rec, precheck=True, pallets=[],
                             script=A.VERBAL, expected_blocked=[]), i))
    return out


def trials_r3(n):
    """R3。V5:南通路のパレットで立ち往生 → 中央通路へ迂回(元 20/20、誤登録 0/20)。学習と、補正した規則(同じ実行部での対照)を交互に。
    V3:口頭で解除された南通路にパレットが残る(事前確認あり / なし)、対照は本当に片付いている。学習だけ(元の値は docs/auto/v3.csv)。
    障害物は元の検証と同じパレット(obstacle place)。"""
    out = []
    for i in range(1, (n or 20) + 1):
        for rec, short in (('learned', 'learned'), ('rule_offset', 'offset')):
            out.append((dict(suite='r3', cond=f'V5_{short}', recognizer=rec, precheck=True, pallets=['south_aisle'],
                             obstacle='pallet', script=['出荷口へ行って'], expected_blocked=['south_aisle']), i))
        if i <= (n or 20) // 2:
            out.append((dict(suite='r3', cond='V3_on_learned', recognizer='learned', precheck=True, pallets=['south_aisle'],
                             obstacle='pallet', script=A.VERBAL, expected_blocked=['south_aisle']), i))
            out.append((dict(suite='r3', cond='V3_off_learned', recognizer='learned', precheck=False, pallets=['south_aisle'],
                             obstacle='pallet', script=A.VERBAL, expected_blocked=['south_aisle']), i))
            out.append((dict(suite='r3', cond='V3_control_learned', recognizer='learned', precheck=True, pallets=[],
                             obstacle='pallet', script=A.VERBAL, expected_blocked=[]), i))
    return out


V3_FILE = '/home/ubuntu/ros2_ws/install/ai_nav/share/ai_nav/config/recognizer_v3.json'


def trials_p1v3(n):
    """P1 と同じ場面を、正規化した認識器(L2n)で。規則の条件は P1(p1.csv)と同じなので測り直さない。"""
    out = []
    for i in range(1, (n or 10) + 1):
        out.append((dict(suite='p1v3', cond='P_learned_v3', recognizer='learned', precheck=True, pallets=['south_aisle'],
                         script=A.VERBAL, expected_blocked=['south_aisle']), i))
        out.append((dict(suite='p1v3', cond='C_learned_v3', recognizer='learned', precheck=True, pallets=[],
                         script=A.VERBAL, expected_blocked=[]), i))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--suite', default='p1', choices=['p1', 'r3', 'p1v3'])
    ap.add_argument('--n', type=int, default=None, help='p1 既定 10、r3 既定 20(V3 はその半分)')
    ap.add_argument('--model', default='qwen2.5:7b')
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--max-errors', type=int, default=3)
    args = ap.parse_args()
    os.makedirs(A.OUT, exist_ok=True)
    path = os.path.join(A.OUT, f'{args.suite}.csv')
    specs = {'p1': lambda: trials(args.n or 10), 'r3': lambda: trials_r3(args.n), 'p1v3': lambda: trials_p1v3(args.n)}[args.suite]()
    todo = [(s, i) for s, i in specs if (s['cond'], i) not in A.load_done(path)]
    print(f'[{args.suite}] 残り {len(todo)} 試行', flush=True)
    if not todo:
        return 0
    if not A.healthy():
        print('シミュレータが応答しない → 再起動を要求(終了コード 75)')
        return A.RESTART
    rc, out = A.sh('ros2 param get /plan_executor recognizer', timeout=30)
    if rc != 0:
        print('plan_executor_v2 で起動していません(recognizer パラメータがない)。run_percep.sh から実行してください')
        return 2
    if args.suite == 'p1v3':
        # 認識器は最初に使うときに読み込まれる(plan_executor_v2.recog)。シミュレータの起動直後に、読み込むファイルを指定する
        A.set_param('recognizer_file', V3_FILE)
    from llm_commander import make_llm
    make_llm(args.model)([{'role': 'user', 'content': '{}'}], {'type': 'object'})
    from ros_link import RosLink
    link = RosLink(args.host, 9090)
    hub = A.Hub(link)
    consecutive = 0
    try:
        for spec, i in todo:
            print(f'[{time.strftime("%H:%M:%S")}] {args.suite} {spec["cond"]} #{i} ...', flush=True)
            try:
                A.place_pallet = _orig_place if spec.get('obstacle') == 'pallet' else place_box
                A.set_param('recognizer', spec['recognizer'])
                row = A.run_trial(link, hub, spec, i, args)
            except A.NeedRestart as e:
                print(f'  再起動が必要: {e}', flush=True)
                return A.RESTART
            except Exception as e:
                print(f'  異常: {e!r}', flush=True)
                consecutive += 1
                if consecutive >= args.max_errors:
                    return A.RESTART
                continue
            A.append_row(path, row)
            print(f'  ok={row["ok"]} t={row.get("t_total")} detected_by={row.get("detected_by")} '
                  f't_first_blocked={row.get("t_first_blocked")} false_register={row.get("false_register")} {row["error"]}',
                  flush=True)
            consecutive = 0 if row['ok'] else consecutive + 1
            if consecutive >= args.max_errors:
                print('  連続失敗 → シミュレータを再起動')
                return A.RESTART
    finally:
        try:
            link.close()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
