"""複数ロボット検証(docs/multi/C*.csv)の集計。

    python analyze_multi.py     # 結果を表示し、docs/multi/summary.md に書く
成功率には Wilson の 95% 区間を付ける。試行は同一シナリオの繰り返しなので、一般化には限りがある。
"""
import csv
import glob
import json
import math
import os
import statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'docs', 'multi')
CONTACT = 0.22     # [m] この距離以下は接触の可能性(ロボットの大きさ約 0.16 × 0.23 m)
NAMES = {'C0': 'C0 協調なし', 'C1a': 'C1a 統制LLM 第1版(避ける通路を出力)', 'C1': 'C1 統制LLM 第2版(通る通路を出力)', 'C2w': 'C2w 環境: 広い通路のみ', 'C2': 'C2 環境: 広い通路 + 車線の通行規則', 'C3': 'C3 ロボット間通信'}


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def load(cond):
    path = os.path.join(OUT, f'{cond}.csv')
    if not os.path.exists(path):
        return []
    with open(path, encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))


def fl(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def summarize(cond, rows):
    n = len(rows)
    ok = [r for r in rows if r['ok_both'] == 'True']
    ts = [fl(r['t_both']) for r in ok if fl(r['t_both']) is not None]
    md = [fl(r['min_dist']) for r in rows if fl(r['min_dist']) is not None]
    lo, hi = wilson(len(ok), n)
    out = {'cond': cond, 'n': n, 'ok': len(ok), 'ci': (lo, hi),
           'ok_r1': sum(r['ok_r1'] == 'True' for r in rows), 'ok_r2': sum(r['ok_r2'] == 'True' for r in rows),
           't_med': st.median(ts) if ts else None, 't_mean': st.mean(ts) if ts else None,
           'md_min': min(md) if md else None, 'contact': sum(m <= CONTACT for m in md),
           'blocked': sum(int(r['n_blocked_r1'] or 0) + int(r['n_blocked_r2'] or 0) for r in rows),
           'false_reg': sum(r['false_reg_r1'] == 'True' or r['false_reg_r2'] == 'True' for r in rows),
           'avoided': sum(bool(r.get('avoid_r1') or r.get('avoid_r2')) for r in rows)}
    calls, valid = 0, 0
    for r in rows:
        try:
            c = json.loads(r.get('coord') or '{}')
        except ValueError:
            c = {}
        if 'calls' in c:
            calls += c['calls']
            valid += bool(c.get('valid'))
    out['llm_calls'], out['llm_valid'] = calls, valid
    return out


def main():
    conds = sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(OUT, 'C*.csv')))
    res = [summarize(c, load(c)) for c in conds]
    lines = ['# 複数ロボット検証の集計(正面すれ違い 2 台)', '',
             '| 条件 | 試行 | 両台到着 (95%区間) | r1 / r2 到着 | 到着時間 中央値 [s] | 最小距離 [m] | 接触の可能性 | 誤登録 | blocked 件数 | 通路を避けた試行 |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for r in res:
        t = f'{r["t_med"]:.1f}' if r['t_med'] is not None else '-'
        md = f'{r["md_min"]:.2f}' if r['md_min'] is not None else '-'
        lines.append(f'| {NAMES.get(r["cond"], r["cond"])} | {r["n"]} | {r["ok"]}/{r["n"]} '
                     f'({r["ci"][0]*100:.0f}〜{r["ci"][1]*100:.0f}%) | {r["ok_r1"]} / {r["ok_r2"]} | {t} | {md} | '
                     f'{r["contact"]}/{r["n"]} | {r["false_reg"]}/{r["n"]} | {r["blocked"]} | {r["avoided"]}/{r["n"]} |')
    for r in res:
        if r['llm_calls']:
            lines.append('')
            lines.append(f'- {NAMES.get(r["cond"], r["cond"])}: 統制 LLM の呼び出し {r["llm_calls"]} 回、'
                         f'検証に通った試行 {r["llm_valid"]}/{r["n"]}')
    lines += ['', f'注: 接触の可能性 = 台間の最小距離が {CONTACT} m 以下。同一シナリオの繰り返しで、位置の初期値は固定。']
    text = '\n'.join(lines) + '\n'
    print(text)
    with open(os.path.join(OUT, 'summary.md'), 'w', encoding='utf-8') as f:
        f.write(text)


if __name__ == '__main__':
    main()
