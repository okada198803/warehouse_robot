"""複数ロボット検証(docs/multi/C*.csv)の集計。

    python analyze_multi.py     # 結果を表示し、docs/multi/summary.md(opp)・summary_<シナリオ>.md・summary_all.md に書く
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
SCEN_NAMES = {'opp': '正面すれ違い', 'same': '同方向(追従)', 'cross': '交差(別の目的地)', 'wait': '通路内で待機中の台'}
CONTACT = 0.22     # [m] この距離以下は接触の可能性(ロボットの大きさ約 0.16 × 0.23 m)
NAMES = {'C0': 'C0 協調なし', 'C1a': 'C1a 統制LLM 第1版(避ける通路を出力)', 'C1': 'C1 統制LLM 第2版(通る通路を出力)', 'C2w': 'C2w 環境: 広い通路のみ', 'C2': 'C2 環境: 広い通路 + 車線の通行規則', 'C3': 'C3 ロボット間通信'}


def label(c):
    b = c.split('_')[0]
    n = NAMES.get(b, b)
    return n + ' [最終コードでの再確認]' if c.endswith('_recheck') else n


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def load(cond):
    path = os.path.join(OUT, f'{cond}.csv')    # cond はファイル名の語幹(C0、C0_same、C0_recheck など)
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


def parse(stem):
    """ファイル名の語幹 → (条件, シナリオ, 再確認か)。C0 → opp、C0_same → same、C0_recheck → opp の再確認。"""
    parts = stem.split('_')
    scn = parts[1] if len(parts) > 1 and parts[1] in SCEN_NAMES else 'opp'
    return parts[0], scn, 'recheck' in parts


def table(res, scn):
    extra = scn != 'opp'     # opp の表は最初の検証のまま(列を足さない)
    head = '| 条件 | 試行 | 両台到着 (95%区間) | r1 / r2 到着 | 到着時間 中央値 [s] |' + (' r1 到着 中央値 [s] |' if extra else '')
    head += ' 最小距離 [m] | 接触の可能性 | ' + ('通行止めを登録した試行' if extra else '誤登録') + ' | blocked 件数 | 通路を避けた試行 |'
    lines = [head, '|' + '---|' * (head.count('|') - 1)]
    for r in res:
        t = f'{r["t_med"]:.1f}' if r['t_med'] is not None else '-'
        md = f'{r["md_min"]:.2f}' if r['md_min'] is not None else '-'
        t1 = ''
        if extra:
            v = [fl(x['t_r1']) for x in r['rows'] if x['ok_r1'] == 'True' and fl(x['t_r1']) is not None]
            t1 = f' {st.median(v):.1f} |' if v else ' - |'
        lines.append(f'| {label(r["stem"])} | {r["n"]} | {r["ok"]}/{r["n"]} '
                     f'({r["ci"][0]*100:.0f}〜{r["ci"][1]*100:.0f}%) | {r["ok_r1"]} / {r["ok_r2"]} | {t} |{t1} {md} | '
                     f'{r["contact"]}/{r["n"]} | {r["false_reg"]}/{r["n"]} | {r["blocked"]} | {r["avoided"]}/{r["n"]} |')
    for r in res:
        if r['llm_calls']:
            lines += ['', f'- {label(r["stem"])}: 統制 LLM の呼び出し {r["llm_calls"]} 回、検証に通った試行 {r["llm_valid"]}/{r["n"]}']
    return lines


def main():
    stems = sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(OUT, 'C*.csv')))
    groups = {}
    for stem in stems:
        cond, scn, _ = parse(stem)
        rows = load(stem)
        r = summarize(cond, rows)
        r['stem'], r['rows'] = stem, rows
        groups.setdefault(scn, []).append(r)
    for scn in SCEN_NAMES:
        res = groups.get(scn)
        if not res:
            continue
        lines = [f'# 複数ロボット検証の集計({SCEN_NAMES[scn]} 2 台)', ''] + table(res, scn)
        lines += ['', f'注: 接触の可能性 = 台間の最小距離が {CONTACT} m 以下。同一シナリオの繰り返しで、位置の初期値は固定。']
        if scn == 'wait':
            lines.append('注: wait では r2 が南通路内で 25 秒待機してから出発する。通行止めの登録は、待機中の r2 を障害物として確認した結果(試行の終了時には塞がっていない)。')
        text = '\n'.join(lines) + '\n'
        print(text)
        with open(os.path.join(OUT, 'summary.md' if scn == 'opp' else f'summary_{scn}.md'), 'w', encoding='utf-8') as f:
            f.write(text)
    # 全シナリオの一覧(再確認の行は除く): 条件 × シナリオの 両台到着 / 到着時間の中央値 / 接触の可能性
    scns = [s for s in SCEN_NAMES if s in groups]
    conds = sorted({parse(r['stem'])[0] for g in groups.values() for r in g})
    lines = ['# 複数ロボット検証 シナリオ別の一覧(両台到着 / 到着時間の中央値 [s] / 接触の可能性)', '',
             '| 条件 | ' + ' | '.join(SCEN_NAMES[s] for s in scns) + ' |', '|' + '---|' * (len(scns) + 1)]
    for c in conds:
        cells = []
        for s in scns:
            r = next((r for r in groups[s] if r['stem'] == (c if s == 'opp' else f'{c}_{s}')), None)
            if r is None:
                cells.append('-')
            else:
                t = f'{r["t_med"]:.1f}' if r['t_med'] is not None else '-'
                cells.append(f'{r["ok"]}/{r["n"]} / {t} / {r["contact"]}/{r["n"]}')
        lines.append(f'| {NAMES.get(c, c)} | ' + ' | '.join(cells) + ' |')
    text = '\n'.join(lines) + '\n'
    print(text)
    with open(os.path.join(OUT, 'summary_all.md'), 'w', encoding='utf-8') as f:
        f.write(text)


if __name__ == '__main__':
    main()
