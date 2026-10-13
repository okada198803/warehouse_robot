"""V8: 到着時間の予測と実測を比べる(ROS 不要)。

    python analyze_eta.py [../docs/auto/v8.csv]

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
                rows.append((r['cond'], float(r['eta_length']), int(r['eta_stops']), 0, float(r['t_total'])))
    return rows


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'docs', 'auto', 'v8.csv')
    rows = load(path)
    samples = [(l, n, c, t) for _, l, n, c, t in rows]
    routes = sorted({r[0] for r in rows})
    L = ['# V8 到着時間の予測', '', f'実測 {len(rows)} 件 / 経路 {len(routes)} 種(障害物なし・停止なしで完了した試行)。', '']

    _, mae, mape = errors(EtaModel(), samples)
    L += ['## 既定のモデル', f'- 平均絶対誤差 {mae:.1f} 秒 / 平均絶対誤差率 {mape:.1%}', '']

    cv_errs, cv_t = [], []
    for held in routes:
        train = [(l, n, c, t) for k, l, n, c, t in rows if k != held]
        test = [(l, n, c, t) for k, l, n, c, t in rows if k == held]
        if len(train) >= 3 and test:
            m = fit(train)
            e, _, _ = errors(m, test)
            cv_errs += e
            cv_t += [t for _, _, _, t in test]
    if cv_errs:
        mae_cv = sum(abs(e) for e in cv_errs) / len(cv_errs)
        mape_cv = sum(abs(e) / t for e, t in zip(cv_errs, cv_t)) / len(cv_errs)
        L += ['## 経路ごとに除いて当てはめ直した場合(当てはめに使っていない経路での予測)',
              f'- 平均絶対誤差 {mae_cv:.1f} 秒 / 平均絶対誤差率 {mape_cv:.1%} / 最大 {max(abs(e) for e in cv_errs):.1f} 秒', '']

    model = fit(samples)
    _, mae_all, mape_all = errors(model, samples)
    L += ['## 全データで当てはめたパラメータ',
          f'- 1 m あたり {model.sec_per_m:.2f} 秒 / 停止 1 回あたり {model.sec_per_stop:.2f} 秒 / 定数 {model.base:.2f} 秒',
          f'- 当てはめに使ったデータでの誤差: 平均絶対誤差 {mae_all:.1f} 秒 / {mape_all:.1%}', '',
          '## 経路別(全データのモデル)', '| 経路 | 経路長 m | 停止 | 実測の平均 s | 予測 s | 誤差 s | n |', '|---|---|---|---|---|---|---|']
    for k in routes:
        sub = [(l, n, c, t) for kk, l, n, c, t in rows if kk == k]
        meas = statistics.mean(t for *_, t in sub)
        pred = model.predict(sub[0][0], sub[0][1])
        L.append(f'| {k.removeprefix("route_")} | {sub[0][0]:.1f} | {sub[0][1]} | {meas:.1f} | {pred:.1f} | {pred - meas:+.1f} | {len(sub)} |')
    text = '\n'.join(L)
    print(text)
    with open(os.path.join(ROOT, 'docs', 'auto', 'v8_summary.md'), 'w', encoding='utf-8') as f:
        f.write(text + '\n')
    with open(os.path.join(HERE, 'eta_model.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(sec_per_m=round(model.sec_per_m, 4), sec_per_stop=round(model.sec_per_stop, 4),
                       base=round(model.base, 4)), f)


if __name__ == '__main__':
    main()
