"""認識層(M8)の分類器を学習し、元の規則(点数 ≥ 16)と比べる。numpy のみ使用。

    python3 tools/train_recognizer.py --train d1.jsonl d2.jsonl --test d3.jsonl --out config/recognizer.json --report r.md

学習とテストは別の乱数の種で生成したファイル(テストの配置は学習に含まれない)。
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from ai_nav.plan_logic import load_places  # noqa: E402
from ai_nav.plan_logic import aisle_scan_hits  # noqa: E402
from ai_nav.scan_features import LIDAR_OFFSET, features  # noqa: E402
import math  # noqa: E402

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
RULE_TH = 16


def load(paths):
    places = {}
    X, y, hits, meta = [], [], [], []
    for p in paths:
        for line in open(p):
            r = json.loads(line)
            w = r['world']
            if w not in places:
                places[w] = load_places(os.path.join(PKG, 'config', f'places_{w}.yaml'))
            a = places[w][r['aisle']]
            X.append(features(r['ranges'], r['angle_min'], r['angle_increment'], r['range_min'], r['range_max'],
                              r['pose'][:2], r['pose'][2], a))
            y.append(r['label'])
            # 規則の比較用:元の規則(ロボット中心から投影)と、LiDAR の位置を補正した規則
            sx = r['pose'][0] + LIDAR_OFFSET * math.cos(r['pose'][2])
            sy = r['pose'][1] + LIDAR_OFFSET * math.sin(r['pose'][2])
            rng = [float('inf') if v is None else v for v in r['ranges']]
            hits.append((r['rule_hits'], aisle_scan_hits(rng, r['angle_min'], r['angle_increment'], r['range_min'],
                                                         r['range_max'], (sx, sy), r['pose'][2], a)))
            o = r['obj']
            lx = r['pose'][0] + LIDAR_OFFSET * math.cos(r['pose'][2]); ly = r['pose'][1] + LIDAR_OFFSET * math.sin(r['pose'][2])
            dist = math.dist((lx, ly), (o['x'], o['y'])) if o else None
            meta.append((w, r['aisle'], (o or {}).get('type', 'none'), dist))
    return np.array(X, float), np.array(y, float), np.array(hits), meta


def train(X, y, hidden=32, epochs=400, lr=0.01, l2=1e-4, seed=0):
    rng = np.random.default_rng(seed)
    n, d = X.shape
    w1 = rng.normal(0, np.sqrt(2 / d), (hidden, d)); b1 = np.zeros(hidden)
    w2 = rng.normal(0, np.sqrt(1 / hidden), hidden); b2 = 0.0
    params = [w1, b1, w2, np.array([b2])]
    m = [np.zeros_like(p) for p in params]; v = [np.zeros_like(p) for p in params]
    pos = y.mean(); cw = np.where(y > 0.5, 0.5 / max(pos, 1e-6), 0.5 / max(1 - pos, 1e-6))   # 不均衡の補正
    for t in range(1, epochs + 1):
        w1, b1, w2, b2a = params
        h = np.maximum(0, X @ w1.T + b1)
        z = h @ w2 + b2a[0]
        p = 1 / (1 + np.exp(-np.clip(z, -30, 30)))
        g = (p - y) * cw / n
        gw2 = h.T @ g + l2 * w2; gb2 = np.array([g.sum()])
        gh = np.outer(g, w2) * (h > 0)
        gw1 = gh.T @ X + l2 * w1; gb1 = gh.sum(0)
        for i, gr in enumerate([gw1, gb1, gw2, gb2]):
            m[i] = 0.9 * m[i] + 0.1 * gr; v[i] = 0.999 * v[i] + 0.001 * gr * gr
            params[i] = params[i] - lr * (m[i] / (1 - 0.9 ** t)) / (np.sqrt(v[i] / (1 - 0.999 ** t)) + 1e-8)
    return params


def predict(params, X):
    w1, b1, w2, b2a = params
    h = np.maximum(0, X @ w1.T + b1)
    return 1 / (1 + np.exp(-np.clip(h @ w2 + b2a[0], -30, 30)))


def table(name, pred, y, meta):
    tp = int(((pred == 1) & (y == 1)).sum()); tn = int(((pred == 0) & (y == 0)).sum())
    fp = int(((pred == 1) & (y == 0)).sum()); fn = int(((pred == 0) & (y == 1)).sum())
    acc = (tp + tn) / len(y)
    rows = [f'| {name} | {acc * 100:.1f}% | {tp} | {fn} | {fp} | {tn} |']
    return rows, (acc, tp, fn, fp, tn)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--train', nargs='+', required=True)
    ap.add_argument('--test', nargs='+', required=True)
    ap.add_argument('--val', nargs='*', default=[], help='しきい値を決める検証用データ(学習・テストに使わない)')
    ap.add_argument('--max-fp', type=float, default=0.02, help='検証用データでの誤登録率の上限(既定 2%%)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--report', required=True)
    args = ap.parse_args()
    Xtr, ytr, _htr, _ = load(args.train)
    Xte, yte, hte, mte = load(args.test)
    mean = Xtr.mean(0); std = Xtr.std(0) + 1e-6
    params = train((Xtr - mean) / std, ytr)
    pte = predict(params, (Xte - mean) / std)
    th, th_note = 0.5, '既定 0.5'
    if args.val:
        # 事前に決めた基準: 検証用データで、誤登録率が max_fp 以下のしきい値のうち、見逃しが最も少ないもの
        Xva, yva, _hva, _ = load(args.val)
        pva = predict(params, (Xva - mean) / std)
        best = None
        for t in np.linspace(0.05, 0.99, 95):
            fp = ((pva >= t) & (yva == 0)).sum() / max((yva == 0).sum(), 1)
            fn = ((pva < t) & (yva == 1)).sum()
            if fp <= args.max_fp and (best is None or fn < best[1]):
                best = (float(t), int(fn), float(fp))
        if best:
            th = best[0]
            th_note = (f'検証用 {len(yva)} 件で誤登録率 {best[2] * 100:.1f}% ≤ {args.max_fp * 100:.0f}% のうち見逃し最少'
                       f'(見逃し {best[1]}/{int(yva.sum())})')
    learned = (pte >= th).astype(int)
    rule = (hte[:, 0] >= RULE_TH).astype(int)
    rule_off = (hte[:, 1] >= RULE_TH).astype(int)
    lines = ['# 認識層(M8)の学習とテスト', '',
             f'- 学習: {len(ytr)} 件({", ".join(os.path.basename(p) for p in args.train)})、障害物あり {int(ytr.sum())}',
             f'- テスト: {len(yte)} 件({", ".join(os.path.basename(p) for p in args.test)})、障害物あり {int(yte.sum())}',
             f'- 規則: 確認帯の点数 ≥ {RULE_TH}。学習: 格子の点数 {Xtr.shape[1] - 4} + 相対位置 4 → 全結合 32 → 1',
             f'- 学習のしきい値: {th:.2f}({th_note})', '',
             '| 判定 | 正解率 | 障害物あり→あり | あり→なし(見逃し) | なし→あり(誤登録) | なし→なし |', '|---|---|---|---|---|---|']
    r1, _ = table('規則(点数 ≥ 16)', rule, yte, mte)
    r1b, _ = table('規則 + LiDAR 位置の補正', rule_off, yte, mte)
    r2, _ = table('学習', learned, yte, mte)
    lines += r1 + r1b + r2 + ['', '## 内訳(テスト)', '', '| ワールド | 通路 | 物体 | 件数 | 規則の誤り | 補正した規則の誤り | 学習の誤り |', '|---|---|---|---|---|---|---|']
    keys = sorted(set(mm[:3] for mm in mte))
    for k in keys:
        idx = [i for i, mm in enumerate(mte) if mm[:3] == k]
        er = int((rule[idx] != yte[idx]).sum()); eo = int((rule_off[idx] != yte[idx]).sum())
        el = int((learned[idx] != yte[idx]).sum())
        lines.append(f'| {k[0]} | {k[1]} | {k[2]} | {len(idx)} | {er} | {eo} | {el} |')
    neg = (yte == 0)
    # 距離別:走行で停止原因を判定するのは、立ち往生した直後(障害物は近い)。遠い障害物は規則の想定外
    lines += ['', '## 障害物までの距離別(テスト。障害物なしは全て含む)', '',
              '| 範囲 | 件数 | 規則 見逃し / 誤登録 | 補正した規則 見逃し / 誤登録 | 学習 見逃し / 誤登録 |', '|---|---|---|---|---|']
    for name, lo, hi in (('障害物なし(帯の外を含む)', None, None), ('障害物あり 1.0 m 未満', 0, 1.0),
                         ('障害物あり 1.0〜2.0 m', 1.0, 2.0), ('障害物あり 2.0 m 以上', 2.0, 99)):
        if lo is None:
            idx = [i for i in range(len(yte)) if yte[i] == 0]
        else:
            idx = [i for i in range(len(yte)) if yte[i] == 1 and lo <= mte[i][3] < hi]
        def c(pr):
            return f"{int(((pr[idx] == 0) & (yte[idx] == 1)).sum())} / {int(((pr[idx] == 1) & (yte[idx] == 0)).sum())}"
        lines.append(f'| {name} | {len(idx)} | {c(rule)} | {c(rule_off)} | {c(learned)} |')
    h0, h1 = hte[:, 0], hte[:, 1]
    lines += ['', f'- 障害物なし(または帯の外)の点数: 元の規則で 16 以上 {int((h0[neg] >= RULE_TH).sum())} 件、'
              f'補正した規則で {int((h1[neg] >= RULE_TH).sum())} 件',
              f'- 障害物ありの点数: 元の規則で 16 未満 {int((h0[~neg] < RULE_TH).sum())} 件、'
              f'補正した規則で {int((h1[~neg] < RULE_TH).sum())} 件']
    open(args.report, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
    w1, b1, w2, b2a = params
    json.dump({'mean': mean.tolist(), 'std': std.tolist(), 'w1': w1.tolist(), 'b1': b1.tolist(),
               'w2': w2.tolist(), 'b2': float(b2a[0]), 'threshold': th,
               'note': '認識層(M8)。tools/train_recognizer.py で学習。特徴量は ai_nav/scan_features.py'},
              open(args.out, 'w'))
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
