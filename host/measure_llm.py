"""LLM の計画精度と応答時間を計測する(ROS 不要。Ollama が必要)。

    cd ~/Desktop/pai_final/host && source .venv/bin/activate
    python measure_llm.py --no-shortest --tag base      # 基準(最短性の検証なし、経験なし)
    python measure_llm.py --tag A                       # A: 最短性の検証あり
    python measure_llm.py --train                       # 経験を作る(experience.json。目的地は計測と別)
    python measure_llm.py --experience on --tag AB      # A + B: 経験を参照(読み取り専用)
    python measure_llm.py --reps 10 --model qwen2.5:7b

結果: ../docs/計測_LLM_<tag>.csv(1 行 = 1 回の判断)と、画面に集計を表示する。
集計項目:
  検証通過率   … LLM が出した goto 案のうち、検証 OK になった割合
  初回通過率   … 判断の最初の LLM 案が検証 OK だった割合
  正答率       … 期待する最終地点と、通行止めを避けたかで判定した割合
  最短率       … 送信した計画が、通行止めを避ける最短経路(許容差内)だった割合
  応答時間     … 判断 1 回あたりの LLM 呼び出し時間の合計(秒)
"""
import argparse
import csv
import os
import statistics
import time

from experience import Experience
from llm_commander import make_llm
from planner import SHORTEST_MARGIN, SHORTEST_TOL, Planner

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.join(HERE, '..', 'docs')
EXPERIENCE_FILE = os.path.join(HERE, 'experience.json')

ALL_AISLES = ['south_aisle', 'middle_aisle', 'north_aisle']
# 停止後に後退した位置(実走の pose に合わせる。通路の中心を使うと再計画の起点が封鎖域内になる)
RETREAT_POSE = {'south_aisle': (1.5, 1.0), 'middle_aisle': (1.5, 2.5), 'north_aisle': (1.5, 3.9)}

# (シナリオ名, 事前の通行止め, 手順, 期待: last=最終地点(None は送信なし), avoid=通らない通路)
# 手順: ('user', 発言) / ('blocked', 停止した通路, 残りの地点)
SCENARIOS = [
    ('S1 出荷口へ', [], [('user', '出荷口へ行って')],
     dict(last='shipping', avoid=[])),
    ('S2 入荷口へ戻る', [], [('user', '入荷口に戻って')],
     dict(last='receiving', avoid=[])),
    ('S3 南封鎖で出荷口', ['south_aisle'], [('user', '出荷口へ行って')],
     dict(last='shipping', avoid=['south_aisle'])),
    ('S4 南・中央封鎖で出荷口', ['south_aisle', 'middle_aisle'], [('user', '出荷口へ行って')],
     dict(last='shipping', avoid=['south_aisle', 'middle_aisle'])),
    ('S5 三通路封鎖で出荷口', ALL_AISLES, [('user', '出荷口へ行って')],
     dict(last=None, avoid=[])),
    ('S6 南の解除のみ', ['south_aisle'], [('user', '南通路は片付いた')],
     dict(last=None, avoid=[], blocked_after=[])),
    ('S7 南の解除と移動', ['south_aisle'], [('user', '南通路は片付いた。出荷口へ行って')],
     dict(last='shipping', avoid=[])),
    ('S8 事務所前へ', [], [('user', '事務所前へ行って')],
     dict(last='office', avoid=[])),
    ('R1 南で停止→再計画', [], [('user', '出荷口へ行って'),
                              ('blocked', 'south_aisle', ['south_aisle', 'shipping'])],
     dict(last='shipping', avoid=['south_aisle'])),
    ('R2 南・中央で停止→再計画', [], [('user', '出荷口へ行って'),
                                  ('blocked', 'south_aisle', ['south_aisle', 'shipping']),
                                  ('blocked', 'middle_aisle', ['middle_aisle', 'shipping'])],
     dict(last='shipping', avoid=['south_aisle', 'middle_aisle'])),
]

# 経験を作るための学習用シナリオ。計測用(出荷口・事務所前)とは目的地を変え、通行止めの状況だけ共通にする
TRAIN = [
    ('T1 南封鎖で充電', ['south_aisle'], [('user', '充電ステーションへ行って')], dict(last='charger', avoid=['south_aisle'])),
    ('T2 南封鎖で東ホール', ['south_aisle'], [('user', '東ホールへ行って')], dict(last='east_hall', avoid=['south_aisle'])),
    ('T3 南・中央封鎖で充電', ['south_aisle', 'middle_aisle'], [('user', '充電ステーションへ行って')],
     dict(last='charger', avoid=['south_aisle', 'middle_aisle'])),
    ('T4 南・中央封鎖で東ホール', ['south_aisle', 'middle_aisle'], [('user', '東ホールへ行って')],
     dict(last='east_hall', avoid=['south_aisle', 'middle_aisle'])),
    ('T5 封鎖なしで東ホール', [], [('user', '東ホールへ行って')], dict(last='east_hall', avoid=[])),
    ('T6 中央封鎖で充電', ['middle_aisle'], [('user', '充電ステーションへ行って')], dict(last='charger', avoid=['middle_aisle'])),
]


# V1: 経験の参照方法の比較用。TRAIN に無い通行止めの組(北通路を含む)で、参照なし/完全一致/類似 を比べる
V1 = [
    ('U1 北封鎖で出荷口', ['north_aisle'], [('user', '出荷口へ行って')],
     dict(last='shipping', avoid=['north_aisle'])),
    ('U2 中央・北封鎖で出荷口', ['middle_aisle', 'north_aisle'], [('user', '出荷口へ行って')],
     dict(last='shipping', avoid=['middle_aisle', 'north_aisle'])),
    ('U3 南・北封鎖で出荷口', ['south_aisle', 'north_aisle'], [('user', '出荷口へ行って')],
     dict(last='shipping', avoid=['south_aisle', 'north_aisle'])),
    ('U4 北封鎖で事務所前', ['north_aisle'], [('user', '事務所前へ行って')],
     dict(last='office', avoid=['north_aisle'])),
    ('U5 中央封鎖で出荷口', ['middle_aisle'], [('user', '出荷口へ行って')],
     dict(last='shipping', avoid=['middle_aisle'])),
    ('U6 北で停止→再計画', [], [('user', '出荷口へ行って'),
                              ('blocked', 'north_aisle', ['north_aisle', 'shipping'])],
     dict(last='shipping', avoid=['north_aisle'])),
]

# V4: 言い換えへの頑健性。同じ意図を別の言い回しで与える(G: 出荷口 / H: 入荷口 / O: 事務所前 / C: 南通路の解除)
PARAPHRASES = {
    'G': ('出荷口へ', 'shipping', ['出荷口へ行って', '出荷口まで移動して', '出荷口に向かってください',
                                 '荷物を出荷口へ運んで', '出荷口へお願いします', '出荷口に行きたい']),
    'H': ('入荷口へ', 'receiving', ['入荷口に戻って', '入荷口へ戻ってください', '入荷口に帰って',
                                  '入荷口まで戻って', '入荷口へ移動して', '入荷口に戻りたい']),
    'O': ('事務所前へ', 'office', ['事務所前へ行って', '事務所の前へ行って', '事務所前まで移動して',
                                 '事務所前に向かってください', '事務所の前までお願いします', '事務所前へ移動']),
}
PARAPHRASES_C = ['南通路は片付いた', '南通路の障害物はなくなった', '南通路は通れるようになった',
                 '南通路の片付けが終わった', '南通路はもう空いている', '南通路を片付けました']


def v4_scenarios():
    """各言い換えを 1 シナリオとする(1 番目は元の言い方)。通行止めなしで目的地を指示する。"""
    out = []
    for g, (label, dest, texts) in PARAPHRASES.items():
        for i, t in enumerate(texts, 1):
            out.append((f'V4{g}{i:02d} {t}', [], [('user', t)], dict(last=dest, avoid=[])))
    for i, t in enumerate(PARAPHRASES_C, 1):
        out.append((f'V4C{i:02d} {t}', ['south_aisle'], [('user', t)],
                    dict(last=None, avoid=[], blocked_after=[])))
    return out


# V4x: 解除の言い方の拡張。規則にない言い方を LLM がどこまで解釈できるか(P: 解除すべき / N: 解除してはいけない)
PARAPHRASES_P = ['南通路はもう大丈夫になった', '南通路の障害物をどかしたよ', '南通路は問題なく通れます',
                 '南通路は元に戻った', '南通路の荷物は片付けておいた', '南通路はもう使って構わない',
                 '南通路を塞いでいたものはなくなりました', '南通路は復旧した']
PARAPHRASES_N = ['南通路はまだ片付いていない', '南通路は通れるようになっていない', '南通路の状況を教えて',
                 '南通路は通れる?', '南通路はまだ使えない']


def v4x_scenarios():
    """P は解除され、LiDAR の確認対象になる(cleared)。N は通行止めのまま。いずれも移動しない。"""
    out = []
    for i, t in enumerate(PARAPHRASES_P, 1):
        out.append((f'V4XP{i:02d} {t}', ['south_aisle'], [('user', t)],
                    dict(last=None, avoid=[], blocked_after=[], cleared_after=['south_aisle'])))
    for i, t in enumerate(PARAPHRASES_N, 1):
        out.append((f'V4XN{i:02d} {t}', ['south_aisle'], [('user', t)],
                    dict(last=None, avoid=[], blocked_after=['south_aisle'], cleared_after=[])))
    return out


def summarize_groups(rows):
    """シナリオ名の先頭(V4G/V4C/V4B など数字の前まで)でまとめた正答率。"""
    import re
    groups = {}
    for r in rows:
        if r['correct'] == '':
            continue
        g = re.match(r'[A-Za-z0-9]*?(?=\d{2}\b|\s|$)', r['scenario']).group(0) or r['scenario']
        groups.setdefault(g, []).append(r['correct'] is True)
    print('\n=== グループ別 正答率 ===')
    for g, v in groups.items():
        print(f'{g:8s} {sum(v)}/{len(v)} = {sum(v) / len(v):.0%}')


def verify_stats(log):
    """判断ログから、LLM 案の検証結果を数える。"""
    ok = ng = 0
    pending = False
    first = None
    for line in log:
        line = line.strip()
        if line.startswith('LLM案'):
            pending = True
        elif pending and line.startswith('検証OK'):
            ok += 1
            pending = False
            first = True if first is None else first
        elif pending and line.startswith('検証NG'):
            ng += 1
            pending = False
            first = False if first is None else first
    return ok, ng, bool(first)


def is_correct(d, planner, exp):
    if 'blocked_after' in exp:
        return (d.plan is None and sorted(planner.blocked) == exp['blocked_after']
                and ('cleared_after' not in exp or sorted(planner.cleared) == exp['cleared_after']))
    if exp['last'] is None:
        return d.plan is None
    targets = (d.plan or {}).get('targets') or []
    return bool(targets) and targets[-1] == exp['last'] and not set(exp['avoid']) & set(targets)


def run_scenario(model, scen, rep, rows, opts=None):
    opts = opts or {}
    name, pre, steps, exp = scen
    timings = []
    base = make_llm(model)

    def llm_fn(messages, schema):
        t0 = time.time()
        out = base(messages, schema)
        timings.append(time.time() - t0)
        return out

    tol = None if opts.get('no_shortest') else SHORTEST_TOL
    planner = Planner('warehouse', llm_fn, shortest_tol=tol, experience=opts.get('experience'),
                      clear_rules=not opts.get('no_clear_rules'), clear_hint=bool(opts.get('clear_hint')),
                      strict_schema=not opts.get('loose_schema'))
    planner.blocked = set(pre)
    for idx, step in enumerate(steps):
        timings.clear()
        if step[0] == 'user':
            d = planner.handle_user(step[1])
        else:
            _, aisle, remaining = step
            pose = RETREAT_POSE[aisle]   # 実走の停止後(後退後)の位置に合わせる
            d = planner.handle_blocked({'state': 'blocked', 'target': remaining[0],
                                        'remaining': remaining, 'near_aisle': aisle,
                                        'pose': list(pose)})
        ok, ng, first_ok = verify_stats(d.log)
        last_step = idx == len(steps) - 1
        route_len, best_len, short = '', '', ''
        targets_sent = (d.plan or {}).get('targets') if d.plan and d.plan.get('action') == 'goto' else None
        if targets_sent:
            problems, total = planner.checker.check_plan(planner._start(), targets_sent, planner.blocked)
            found = planner._best(targets_sent)
            if not problems and found:
                route_len, best_len = round(total, 1), round(found[1], 1)
                short = total <= found[1] * (1 + SHORTEST_TOL) + SHORTEST_MARGIN
        rows.append({
            'scenario': name, 'rep': rep, 'step': idx + 1,
            'kind': 'replan' if idx > 0 else 'first',
            'source': d.source,
            'targets': '→'.join((d.plan or {}).get('targets', [])) if d.plan else '',
            'llm_calls': len(timings), 'verified_ok': ok, 'verified_ng': ng,
            'first_try_ok': first_ok, 'llm_time_s': round(sum(timings), 2),
            'correct': is_correct(d, planner, exp) if last_step else '',
            'route_len': route_len, 'best_len': best_len, 'shortest': short,
            'used_experience': any('過去の成功例を参照' in l for l in d.log),
        })
        if opts.get('train') and last_step and rows[-1]['correct'] is True and short is True:
            planner.on_status({'state': 'done'})   # 最短で正しい計画だけを、到着した経験として保存


def summarize(rows, scenarios=None):
    print('\n=== シナリオ別 ===')
    print(f'{"シナリオ":24s} {"正答率":>8s} {"検証通過率":>10s} {"平均応答(秒)":>12s}')
    for name, *_ in (scenarios or SCENARIOS):
        rs = [r for r in rows if r['scenario'] == name]
        final = [r for r in rs if r['correct'] != '']
        ok = sum(1 for r in final if r['correct'] is True)
        ver = sum(r['verified_ok'] for r in rs)
        tot = ver + sum(r['verified_ng'] for r in rs)
        rate = f'{ver / tot:.0%}' if tot else '-'
        t = statistics.mean(r['llm_time_s'] for r in rs) if rs else 0
        print(f'{name:24s} {ok}/{len(final):<6} {rate:>10s} {t:>12.1f}')

    all_ver = sum(r['verified_ok'] for r in rows)
    all_tot = all_ver + sum(r['verified_ng'] for r in rows)
    firsts = [r for r in rows if r['kind'] == 'first' and r['verified_ok'] + r['verified_ng'] > 0]
    first_rate = sum(1 for r in firsts if r['first_try_ok']) / len(firsts) if firsts else 0
    replans = [r for r in rows if r['kind'] == 'replan']
    print('\n=== 全体 ===')
    print(f'LLM 案の検証通過率: {all_ver}/{all_tot} = {all_ver / all_tot:.0%}' if all_tot else '検証なし')
    print(f'初回案の通過率: {first_rate:.0%}')
    if replans:
        print(f'再計画の応答時間: 平均 {statistics.mean(r["llm_time_s"] for r in replans):.1f} 秒'
              f'(中央値 {statistics.median(r["llm_time_s"] for r in replans):.1f} 秒、{len(replans)} 回)')
    cmp_rows = [r for r in rows if r['shortest'] != '']
    if cmp_rows:
        n_short = sum(1 for r in cmp_rows if r['shortest'] is True)
        excess = [(r['route_len'] / r['best_len'] - 1) * 100 for r in cmp_rows if r['best_len']]
        print(f'最短率: {n_short}/{len(cmp_rows)} = {n_short / len(cmp_rows):.0%}'
              f'(最短に対する超過 平均 {statistics.mean(excess):.1f}%)')
    used = [r for r in rows if r['used_experience']]
    if used:
        print(f'経験を参照した判断: {len(used)} 件')
    fin = [r for r in rows if r['correct'] != '']
    print(f'全体の正答率: {sum(r["correct"] is True for r in fin)}/{len(fin)}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--reps', type=int, default=5)
    ap.add_argument('--model', default='qwen2.5:7b')
    ap.add_argument('--tag', default='base', help='結果 CSV の名前(計測_LLM_<tag>.csv)')
    ap.add_argument('--no-shortest', action='store_true', help='最短性の検証を行わない(基準用)')
    ap.add_argument('--experience', choices=['off', 'on', 'similar'], default='off',
                    help='on: experience.json を参照(読み取り専用。通行止めの組が一致するもの)/ similar: 一致が無ければ類似を参照')
    ap.add_argument('--suite', choices=['base', 'v1', 'v4', 'v4x'], default='base', help='base: S/R シナリオ / v1: 経験の比較用 / v4: 言い換え')
    ap.add_argument('--no-clear-rules', action='store_true', help='解除の語の規則を使わず LLM だけで解釈する(比較用)')
    ap.add_argument('--clear-hint', action='store_true', help='プロンプトに解除の言い方の例と否定の扱いを足す')
    ap.add_argument('--strict-schema', action='store_true', help='(既定。互換のため残す)')
    ap.add_argument('--loose-schema', action='store_true', help='blocked_add / blocked_clear を必須にしない(旧仕様。比較用)')
    ap.add_argument('--train', action='store_true', help='学習用シナリオで experience.json を作る(1 回ずつ)')
    args = ap.parse_args()

    opts = {'no_shortest': args.no_shortest, 'no_clear_rules': args.no_clear_rules, 'clear_hint': args.clear_hint,
            'loose_schema': args.loose_schema}
    scenarios = {'base': SCENARIOS, 'v1': V1, 'v4': v4_scenarios(), 'v4x': v4x_scenarios()}[args.suite]
    reps = args.reps
    if args.train:
        scenarios, reps, args.tag = TRAIN, 1, 'train'
        if os.path.exists(EXPERIENCE_FILE):
            os.remove(EXPERIENCE_FILE)   # 作り直す
        opts.update(train=True, experience=Experience(EXPERIENCE_FILE))
    elif args.experience in ('on', 'similar'):
        opts['experience'] = Experience(EXPERIENCE_FILE, read_only=True, similar=args.experience == 'similar')
        print(f'経験 {len(opts["experience"])} 件を参照(読み取り専用、{args.experience})')

    # 初回の呼び出しはモデルの読み込みで遅いため、計測前に 1 回温めておく
    run_scenario(args.model, SCENARIOS[1], 0, [])

    rows = []
    for scen in scenarios:
        for rep in range(1, reps + 1):
            print(f'{scen[0]} 回{rep}/{reps}', flush=True)
            run_scenario(args.model, scen, rep, rows, opts)

    out = os.path.join(DOCS, f'計測_LLM_{args.tag}.csv')
    os.makedirs(DOCS, exist_ok=True)
    with open(out, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f'\n保存: {os.path.normpath(out)}')
    summarize(rows, scenarios)
    if args.suite in ('v4', 'v4x'):
        summarize_groups(rows)
    if args.train:
        print(f'経験 {len(opts["experience"])} 件を {os.path.normpath(EXPERIENCE_FILE)} に保存')


if __name__ == '__main__':
    main()
