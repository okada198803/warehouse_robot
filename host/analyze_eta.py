"""V8: 到着時間の予測と実測を比べる(ROS 不要)。

    python analyze_eta.py [../docs/auto/v8.csv [../docs/auto/v8b.csv]]
(v8b = 荷物ありの経路。あれば、荷物 1 つあたりの時間も当てはめる)

1. 既定のモデル(eta.EtaModel)の誤差
2. 経路ごとに除いて当てはめ直した場合の誤差(leave-one-route-out)。当てはめに使っていない経路での予測の精度
3. 全データで当てはめたパラメータを host/eta_model.json に保存(llm_commander が読む)
結果: docs/auto/v8_summary.md
"""
import csv
import json
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from eta import EtaModel, errors, fit  # noqa: E402

ROOT = os.path.normpath(os.path.join(HERE, '..'))


def load(path):
    rows = []
    with open(path, encoding='utf-8-sig') as f:
        for r in csv.DictReader(f):
            if r['ok'] == 'True' and r['t_total'] and r['eta_length'] and r['n_blocked'] in ('0', ''):
                rows.append((r['cond'], float(r['eta_length']), int(r['eta_stops']), int(r.get('eta_cargo') or 0),
                             float(r['t_total'])))
    return rows


def cv(rows):
    """経路ごとに除いて当てはめ直し、除いた経路を予測した誤差。(誤差の一覧, 実測の一覧)"""
    errs, ts = [], []
    for held in sorted({r[0] for r in rows}):
        train = [(l, n, c, t) for k, l, n, c, t in rows if k != held]
        test = [(l, n, c, t) for k, l, n, c, t in rows if k == held]
        if len(train) >= 3 and test:
            e, _, _ = errors(fit(train), test)
            errs += e
            ts += [t for _, _, _, t in test]
    return errs, ts


def stat(errs, ts):
    return (f'平均絶対誤差 {sum(abs(e) for e in errs) / len(errs):.1f} 秒 / '
            f'平均絶対誤差率 {sum(abs(e) / t for e, t in zip(errs, ts)) / len(errs):.1%} / 最大 {max(abs(e) for e in errs):.1f} 秒')


def main():
    p8 = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'docs', 'auto', 'v8.csv')
    pb = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, 'docs', 'auto', 'v8b.csv')
    plain = load(p8)
    cargo = load(pb) if os.path.exists(pb) else []
    rows = plain + cargo
    samples = [(l, n, c, t) for _, l, n, c, t in rows]
    routes = sorted({r[0] for r in rows})
    L = ['# V8 到着時間の予測', '',
         f'実測 {len(rows)} 件 / 経路 {len(routes)} 種(障害物なしで完了した試行)。うち荷物なし {len(plain)} 件(V8)、荷物あり {len(cargo)} 件(V8b)。', '']

    old = EtaModel(load_wait=2.0)
    _, mae, mape = errors(old, [(l, n, c, t) for _, l, n, c, t in plain])
    L += ['## 荷物なし: 初期値のモデル(荷物の時間 2 秒固定)', f'- 平均絶対誤差 {mae:.1f} 秒 / 平均絶対誤差率 {mape:.1%}', '']
    if cargo:
        e0, _, _ = errors(fit([(l, n, c, t) for _, l, n, c, t in plain]), [(l, n, c, t) for _, l, n, c, t in cargo])
        t0 = [t for *_, t in cargo]
        L += ['## 荷物あり: 荷物なしの実測だけで当てはめたモデルでの予測(荷物の時間は 2 秒固定)',
              f'- {stat(e0, t0)}(符号は予測 − 実測。平均 {sum(e0) / len(e0):+.1f} 秒)', '']

    for title, sub in (('荷物なしの経路', plain), ('荷物ありの経路', cargo), ('全経路', rows)):
        if not sub:
            continue
        errs, ts = cv(sub)
        if errs:
            L += [f'## 経路ごとに除いて当てはめ直した場合: {title}(当てはめに使っていない経路での予測)',
                  f'- {stat(errs, ts)}', '']

    model = fit(samples)
    _, mae_all, mape_all = errors(model, samples)
    L += ['## 全データで当てはめたパラメータ',
          f'- 1 m あたり {model.sec_per_m:.2f} 秒 / 停止 1 回あたり {model.sec_per_stop:.2f} 秒 / 定数 {model.base:.2f} 秒'
          f' / 荷物 1 つあたり {model.load_wait:.2f} 秒',
          f'- 当てはめに使ったデータでの誤差: 平均絶対誤差 {mae_all:.1f} 秒 / {mape_all:.1%}', '',
          '## 経路別(全データのモデル)', '| 経路 | 経路長 m | 停止 | 荷物 | 実測の平均 s | 予測 s | 誤差 s | n |', '|---|---|---|---|---|---|---|---|']
    for k in routes:
        sub = [(l, n, c, t) for kk, l, n, c, t in rows if kk == k]
        meas = statistics.mean(t for *_, t in sub)
        pred = model.predict(sub[0][0], sub[0][1], sub[0][2])
        L.append(f'| {k.removeprefix("route_")} | {sub[0][0]:.1f} | {sub[0][1]} | {sub[0][2]} | {meas:.1f} | {pred:.1f} | {pred - meas:+.1f} | {len(sub)} |')
    text = '\n'.join(L)
    print(text)
    with open(os.path.join(ROOT, 'docs', 'auto', 'v8_summary.md'), 'w', encoding='utf-8') as f:
        f.write(text + '\n')
    with open(os.path.join(HERE, 'eta_model.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(sec_per_m=round(model.sec_per_m, 4), sec_per_stop=round(model.sec_per_stop, 4),
                       base=round(model.base, 4), load_wait=round(model.load_wait, 4)), f)


if __name__ == '__main__':
    main()
