"""V7: record_scans.py の記録から、LiDAR 事前確認のしきい値・帯の大きさに対する感度を計算する(ROS 不要)。

    python analyze_v7.py [../ros2_ws/v7_scans.json]

検知(正): その通路にパレットがあり、ロボットがその通路の入口に向いている記録(label が通路 id で始まる)で、帯内の点数 >= しきい値
          (他の通路の入口にいる記録は、壁に遮られて見えないため正から除く)
誤検知(負): その通路にパレットがない全ての記録(パレットなし/他の通路にある)で、帯内の点数 >= しきい値
結果: docs/auto/v7_summary.md と docs/auto/v7_sensitivity.csv
"""
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'ai_nav'))
from ai_nav.plan_logic import aisle_scan_hits, load_places  # noqa: E402

PLACES = os.path.join(ROOT, 'ros2_ws', 'src', 'ai_nav', 'config', 'places_warehouse.yaml')
AISLES = ['south_aisle', 'middle_aisle', 'north_aisle']
CURRENT = dict(half_width=0.3, half_length=1.2, min_hits=3)


def hits_of(rec, aisle, places, hw, hl):
    ranges = [math.inf if r is None else r for r in rec['ranges']]
    x, y, yaw = rec['robot']
    return aisle_scan_hits(ranges, rec['angle_min'], rec['angle_increment'], rec['range_min'],
                           rec['range_max'], (x, y), yaw, places[aisle], hl, hw)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'ros2_ws', 'v7_scans.json')
    with open(path, encoding='utf-8') as f:
        recs = json.load(f)
    places = load_places(PLACES)
    widths = [0.2, 0.3, 0.4, 0.5]
    lengths = [0.8, 1.2, 1.6]
    thresholds = [1, 3, 8, 12, 15, 17, 20]

    # (hw, hl) ごとに、全記録 × 全通路の点数を一度だけ計算する
    table = {}
    for hw in widths:
        for hl in lengths:
            table[(hw, hl)] = [(r, a, hits_of(r, a, places, hw, hl)) for r in recs for a in AISLES]

    def rates(hw, hl, thr, extra=lambda r, a: True):
        pos = [h for r, a, h in table[(hw, hl)] if r['cond'] == a and r['label'].startswith(a) and extra(r, a)]
        neg = [h for r, a, h in table[(hw, hl)] if r['cond'] != a and extra(r, a)]
        tp = sum(h >= thr for h in pos)
        fp = sum(h >= thr for h in neg)
        return tp, len(pos), fp, len(neg)

    rows = []
    for hw in widths:
        for hl in lengths:
            for thr in thresholds:
                tp, np_, fp, nn = rates(hw, hl, thr)
                rows.append(dict(half_width=hw, half_length=hl, min_hits=thr, detect=tp, positives=np_,
                                 false_alarm=fp, negatives=nn,
                                 detect_rate=round(tp / np_, 3) if np_ else '',
                                 false_rate=round(fp / nn, 3) if nn else ''))
    out_dir = os.path.join(ROOT, 'docs', 'auto')
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'v7_sensitivity.csv'), 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    L = ['# V7 LiDAR しきい値の感度', '',
         f'記録 {len(recs)} 件(条件: {sorted({r["cond"] for r in recs})})。'
         f'現行設定: 帯の幅 ±{CURRENT["half_width"]} m、長さ ±{CURRENT["half_length"]} m、しきい値 {CURRENT["min_hits"]} 点。', '']
    tp, np_, fp, nn = rates(CURRENT['half_width'], CURRENT['half_length'], CURRENT['min_hits'])
    L += ['## 現行設定', f'- 検知: {tp}/{np_} = {tp / np_:.1%}' if np_ else '- 検知: 記録なし',
          f'- 誤検知: {fp}/{nn} = {fp / nn:.1%}' if nn else '- 誤検知: 記録なし', '']

    # 点数の分布
    pos = sorted(h for r, a, h in table[(0.3, 1.2)] if r['cond'] == a and r['label'].startswith(a))
    neg = sorted(h for r, a, h in table[(0.3, 1.2)] if r['cond'] != a)
    if pos and neg:
        L += ['## 点数の分布(現行の帯)',
              f'- パレットあり: 最小 {pos[0]}、中央値 {pos[len(pos) // 2]}、最大 {pos[-1]}',
              f'- パレットなし: 最小 {neg[0]}、中央値 {neg[len(neg) // 2]}、最大 {neg[-1]}', '']

    if pos and neg:
        L += [f'- 分離: パレットなしの最大 {neg[-1]} 点 < パレットありの最小 {pos[0]} 点 なら、その間のしきい値で誤りなく分けられる', '']
    L += ['## しきい値(帯は現行)', '| しきい値 | 検知率 | 誤検知率 |', '|---|---|---|']
    for thr in thresholds:
        tp, np_, fp, nn = rates(0.3, 1.2, thr)
        L.append(f'| {thr} 点 | {tp}/{np_} ({tp / np_:.0%}) | {fp}/{nn} ({fp / nn:.0%}) |' if np_ and nn else f'| {thr} | — | — |')

    L += ['', '## 帯の幅(しきい値 3、長さ ±1.2 m)', '| 幅 | 検知率 | 誤検知率 |', '|---|---|---|']
    for hw in widths:
        tp, np_, fp, nn = rates(hw, 1.2, 3)
        if np_ and nn:
            L.append(f'| ±{hw} m | {tp}/{np_} ({tp / np_:.0%}) | {fp}/{nn} ({fp / nn:.0%}) |')

    L += ['', '## 帯の長さ(しきい値 3、幅 ±0.3 m)', '| 長さ | 検知率 | 誤検知率 |', '|---|---|---|']
    for hl in lengths:
        tp, np_, fp, nn = rates(0.3, hl, 3)
        if np_ and nn:
            L.append(f'| ±{hl} m | {tp}/{np_} ({tp / np_:.0%}) | {fp}/{nn} ({fp / nn:.0%}) |')

    # 通路までの距離別の検知(西側から、通路中心 x=3.5 まで)
    L += ['', '## 検知率と通路までの距離(現行設定。ロボットが通路の西側にいる記録)',
          '| 通路中心までの距離 | 検知 | 点数の中央値 |', '|---|---|---|']
    buckets = [(0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 3.0), (3.0, 9.0)]
    for lo, hi in buckets:
        hs = [h for r, a, h in table[(0.3, 1.2)]
              if r['cond'] == a and r['label'].startswith(a) and r['robot'][0] < places[a]['x'] and lo <= places[a]['x'] - r['robot'][0] < hi]
        if hs:
            hs_sorted = sorted(hs)
            L.append(f'| {lo}〜{hi} m | {sum(h >= 3 for h in hs)}/{len(hs)} | {hs_sorted[len(hs) // 2]} |')

    # 誤検知の内訳(あれば)
    bad = [(r['cond'], r['label'], a, h) for r, a, h in table[(0.3, 1.2)] if r['cond'] != a and h >= 3]
    L += ['', '## 現行設定での誤検知(パレットがないのに 3 点以上)']
    L += [f'- パレット={c} / 位置={lab} / 判定した通路={a} / {h} 点' for c, lab, a, h in bad[:20]] or ['- なし']
    text = '\n'.join(L)
    print(text)
    with open(os.path.join(out_dir, 'v7_summary.md'), 'w', encoding='utf-8') as f:
        f.write(text + '\n')


if __name__ == '__main__':
    main()
