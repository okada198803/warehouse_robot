"""R1c:学習した認識器の汎化の比較(docs/perception/R1c_計画.md)。4 つの版を学習し、4 つのテストで見逃し・誤登録を比べる。

    python3 tools/compare_recognizers.py --data ../../../docs/perception/data --out-dir ../../../docs/perception/model --report r.md
"""
import argparse
import json
import os

import numpy as np

from train_recognizer import RULE_TH, load, predict, train

TRAIN_NARROW = ['scan_warehouse_s1', 'scan_warehouse_s2', 'scan_warehouse_s4', 'scan_warehouse_s6', 'scan_warehouse_s7']
VARIANTS = [   # (名前, 学習, 検証, 正規化)
    ('L2', TRAIN_NARROW, ['scan_warehouse_s8'], False),
    ('L2n', TRAIN_NARROW, ['scan_warehouse_s8'], True),
    ('L3', TRAIN_NARROW + ['scan_warehouse_wide_s1'], ['scan_warehouse_s8', 'scan_warehouse_wide_s2'], False),
    ('L3n', TRAIN_NARROW + ['scan_warehouse_wide_s1'], ['scan_warehouse_s8', 'scan_warehouse_wide_s2'], True),
]
TESTS = [('狭い通路・通路内', ['scan_warehouse_s3']), ('狭い通路・ホールから', ['scan_warehouse_s5']),
         ('幅の広いワールド(1.5 m)', ['scan_warehouse_wide_s3']), ('幅 1.2 m(学習にない幅)', ['scan_warehouse_mid_s1'])]


def threshold(params, mean, std, Xva, yva, max_fp=0.02):
    pva = predict(params, (Xva - mean) / std)
    best = None
    for t in np.linspace(0.05, 0.99, 95):
        fp = ((pva >= t) & (yva == 0)).sum() / max((yva == 0).sum(), 1)
        fn = ((pva < t) & (yva == 1)).sum()
        if fp <= max_fp and (best is None or fn < best[1]):
            best = (float(t), int(fn), float(fp))
    return best[0] if best else 0.5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--report', required=True)
    a = ap.parse_args()
    f = lambda names: [os.path.join(a.data, n + '.jsonl') for n in names]   # noqa: E731
    tests = [(n, p) for n, p in TESTS if all(os.path.exists(x) for x in f(p))]
    res, ths = {}, {}
    for name, tr, va, norm in VARIANTS:
        Xtr, ytr, _, _ = load(f(tr), norm)
        mean = Xtr.mean(0); std = Xtr.std(0) + 1e-6
        params = train((Xtr - mean) / std, ytr)
        Xva, yva, _, _ = load(f(va), norm)
        th = threshold(params, mean, std, Xva, yva); ths[name] = th
        w1, b1, w2, b2a = params
        json.dump({'mean': mean.tolist(), 'std': std.tolist(), 'w1': w1.tolist(), 'b1': b1.tolist(), 'w2': w2.tolist(),
                   'b2': float(b2a[0]), 'threshold': th, 'norm': norm,
                   'note': f'R1c の {name}(tools/compare_recognizers.py)。学習 {",".join(tr)}、検証 {",".join(va)}'},
                  open(os.path.join(a.out_dir, f'recognizer_r1c_{name}.json'), 'w'))
        for tn, tp in tests:
            Xte, yte, hte, _ = load(f(tp), norm)
            pr = (predict(params, (Xte - mean) / std) >= th).astype(int)
            res[(name, tn)] = (int(((pr == 0) & (yte == 1)).sum()), int(((pr == 1) & (yte == 0)).sum()),
                               int(yte.sum()), int((yte == 0).sum()))
            ro = (hte[:, 1] >= RULE_TH).astype(int)
            res[('補正した規則', tn)] = (int(((ro == 0) & (yte == 1)).sum()), int(((ro == 1) & (yte == 0)).sum()),
                                     int(yte.sum()), int((yte == 0).sum()))
    names = ['補正した規則'] + [v[0] for v in VARIANTS]
    L = ['# R1c:学習した認識器の汎化の比較', '', '計画は R1c_計画.md。各セルは 見逃し / 誤登録(誤登録率)。', '',
         '| テスト(あり / なし) | ' + ' | '.join(names) + ' |', '|---|' + '---|' * len(names)]
    for tn, _ in tests:
        npos, nneg = res[('L2', tn)][2], res[('L2', tn)][3]
        row = [f'{tn}({npos} / {nneg})']
        for n in names:
            m, fp, _, ng = res[(n, tn)]
            row.append(f'{m} / {fp}({fp / max(ng, 1) * 100:.1f}%)')
        L.append('| ' + ' | '.join(row) + ' |')
    L += ['', '| 版 | しきい値 | 誤登録率の最大 | 狭い通路・通路内の見逃し / 誤登録 |', '|---|---|---|---|']
    for n in names[1:]:
        mx = max(res[(n, tn)][1] / max(res[(n, tn)][3], 1) for tn, _ in tests)
        m0, f0, _, _ = res[(n, tests[0][0])]
        L.append(f'| {n} | {ths[n]:.2f} | {mx * 100:.1f}% | {m0} / {f0} |')
    open(a.report, 'w', encoding='utf-8').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
