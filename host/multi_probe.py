"""2台構成の動作確認。rosbridge(既定 localhost:9091)経由で各台に指示を出し、位置と状態を記録する。

    python multi_probe.py r1            # r1 だけ入荷口→出荷口へ(r2 は動かない確認)
    python multi_probe.py both          # 正面すれ違い: r1 入荷口→出荷口、r2 出荷口→入荷口を同時に
    python multi_probe.py both --secs 150
最後に、各台の最終状態・最小の台間距離・停止(動かなかった時間)を表示する。
"""
import argparse
import math
import time

import roslibpy

from ros_link_ns import NsLink as RosLink

GOALS = {'r1': 'shipping', 'r2': 'receiving'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['r1', 'r2', 'both'])
    ap.add_argument('--port', type=int, default=9091)
    ap.add_argument('--secs', type=float, default=100.0)
    ap.add_argument('--via', default='south_aisle', help='経由する通路(両台とも同じ通路を使わせる)')
    args = ap.parse_args()

    names = ['r1', 'r2'] if args.mode == 'both' else [args.mode]
    links = {ns: RosLink(port=args.port, ns=ns) for ns in ('r1', 'r2')}
    pos, state = {}, {ns: {} for ns in links}
    for ns, link in links.items():
        topic = roslibpy.Topic(link.ros, f'/{ns}/odom', 'nav_msgs/msg/Odometry')
        topic.subscribe(lambda m, ns=ns: pos.__setitem__(
            ns, (m['pose']['pose']['position']['x'], m['pose']['pose']['position']['y'])))
        link.on_status(lambda st, ns=ns: state[ns].update(st))
    time.sleep(2.0)
    print('initial', {k: tuple(round(c, 2) for c in v) for k, v in pos.items()})

    t0 = time.time()
    for ns in names:
        links[ns].send_plan({'action': 'goto', 'targets': [args.via, GOALS[ns]]})
    mind, last, still = 99.0, dict(pos), {ns: 0.0 for ns in names}
    done = {}
    while time.time() - t0 < args.secs and len(done) < len(names):
        time.sleep(1.0)
        if len(pos) == 2:
            mind = min(mind, math.dist(pos['r1'], pos['r2']))
        for ns in names:
            if ns in pos and ns in last and math.dist(pos[ns], last[ns]) < 0.02:
                still[ns] += 1.0
            else:
                still[ns] = 0.0
            last[ns] = pos.get(ns, last.get(ns))
            if state[ns].get('state') in ('done', 'blocked', 'failed') and ns not in done:
                done[ns] = (round(time.time() - t0, 1), state[ns].get('state'))
        if int(time.time() - t0) % 5 == 0:
            print(f'{time.time() - t0:5.0f}s', {ns: (tuple(round(c, 2) for c in pos.get(ns, (0, 0))),
                                                      state[ns].get('state'), state[ns].get('target'),
                                                      f'still={still.get(ns, 0):.0f}s') for ns in links})
    print('--- result')
    for ns in names:
        print(ns, 'finished' if ns in done else 'timeout', done.get(ns), state[ns].get('state'),
              state[ns].get('near_aisle'), state[ns].get('cause'))
    print('min distance between robots [m]:', round(mind, 2))
    if args.mode != 'both':
        other = 'r2' if names[0] == 'r1' else 'r1'
        print(f'{other} moved [m]:', round(math.dist(pos[other], (0.8, 0.8) if other == 'r1' else (6.2, 0.8)), 2))
    for link in links.values():
        link.close()


if __name__ == '__main__':
    main()
