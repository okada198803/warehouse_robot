"""docs/auto/*.csv(auto_sim.py の結果)を集計して表示し、docs/auto/summary.md に保存する。"""
import csv
import json
import os
import statistics as st

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'auto')


def load(name):
    p = os.path.join(OUT, f'{name}.csv')
    if not os.path.exists(p):
        return []
    with open(p, encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))


def nums(rows, key, only_ok=True):
    out = []
    for r in rows:
        if only_ok and r['ok'] != 'True':
            continue
        try:
            out.append(float(r[key]))
        except (KeyError, ValueError):
            pass
    return out


def stat(v):
    if not v:
        return '—'
    sd = st.stdev(v) if len(v) > 1 else 0.0
    return f'平均 {st.mean(v):.1f} s(SD {sd:.1f}、最小 {min(v):.1f}、最大 {max(v):.1f}、n={len(v)})'


def count(rows, pred):
    return sum(1 for r in rows if pred(r))


def main():
    lines = []
    p = lines.append

    v2 = load('v2')
    if v2:
        p('## V2 フォールバック(スタブ LLM が常に通行止めの経路を返す場合)')
        p(f'- 試行 {len(v2)} 回 / フォールバック発動 {count(v2, lambda r: r["fallback_used"] == "True")} 回 / '
          f'到着 {count(v2, lambda r: r["ok"] == "True")} 回')
        p(f'- 到着までの時間: {stat(nums(v2, "t_total"))}')
        p(f'- 送信された計画: {sorted({r["plans"] for r in v2})}')

    v3 = load('v3')
    if v3:
        p('\n## V3 口頭解除の事前確認(D: パレットが残っている / E: 実際に撤去済み)')
        for cond, label in (('D_precheck_on', 'D 事前確認 ON'), ('D_precheck_off', 'D 事前確認 OFF'),
                            ('D_control_cleared', 'D 対照: 撤去済み(事前確認 ON)'),
                            ('E_unconfirmed', 'E 原因を確認できない停止(scan_min_hits=999)')):
            rs = [r for r in v3 if r['cond'] == cond]
            if not rs:
                continue
            p(f'- **{label}**: {len(rs)} 回 / 到着 {count(rs, lambda r: r["ok"] == "True")} 回 / '
              f'誤登録あり {count(rs, lambda r: r["false_register"] == "True")} 回')
            if cond.startswith('D'):
                det = count(rs, lambda r: r['n_blocked'] not in ('', '0'))
                p(f'  - 障害物を検知(blocked 報告): {det}/{len(rs)} 回(検知手段: '
                  f'{ {k: count(rs, lambda r, k=k: r["detected_by"] == k) for k in ("precheck", "stall")} })')
                p(f'  - 検知までの時間(計画送信から blocked まで): {stat(nums(rs, "t_first_blocked", only_ok=False))}')
            elif cond == 'E_unconfirmed':
                p(f'  - 期待どおり(通行止めに登録せず 1 回再試行し、2 回目で停止): {count(rs, lambda r: r["ok"] == "True")}/{len(rs)}')
                p(f'  - 誤って通行止めに登録: {count(rs, lambda r: r["final_blocked"] != "")}/{len(rs)}')
                continue
            else:
                p(f'  - 誤って棄却(blocked 報告)した回数: {count(rs, lambda r: r["n_blocked"] not in ("", "0"))}/{len(rs)}')
            p(f'  - 到着までの時間: {stat(nums(rs, "t_total"))}')

    v5 = load('v5')
    if v5:
        p('\n## V5a/V6 南通路のパレット → 中央通路へ迂回(実走)')
        mid = 0
        for r in v5:
            try:
                bl = json.loads(r['blocks'])
            except ValueError:
                bl = []
            if any(b.get('near') == 'middle_aisle' for b in bl) or 'middle_aisle' in r['final_blocked']:
                mid += 1
        p(f'- 試行 {len(v5)} 回 / 到着 {count(v5, lambda r: r["ok"] == "True")} 回 / 中央通路の誤検知(誤登録) {mid} 回 / '
          f'誤登録あり(南以外) {count(v5, lambda r: r["false_register"] == "True")} 回')
        p(f'- 到着までの時間: {stat(nums(v5, "t_total"))}')
        p(f'- 初回の計画: {sorted({r["first_plan"] for r in v5})}')
        p(f'- 失敗の内訳: {sorted({r["error"] for r in v5 if r["error"]}) or "なし"}')

    text = '\n'.join(lines) if lines else '結果がありません(docs/auto/*.csv)'
    print(text)
    with open(os.path.join(OUT, 'summary.md'), 'w', encoding='utf-8') as f:
        f.write('# 自動検証の集計\n\n' + text + '\n')


if __name__ == '__main__':
    main()
