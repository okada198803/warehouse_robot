"""LiDAR 事前確認のしきい値(帯内の点数)を、記録から自動で選ぶ(ROS 不要)。

入力: パレットあり(pos)・なし(neg)の記録ごとの、帯内の点数。
方法: 検知漏れ率 + 誤検知率 が最小のしきい値のうち、両側の余裕(マージン)が最大になるものを選ぶ。
      パレットなしの最大点数 < パレットありの最小点数 なら、その間の中央(分離できる場合)。
評価: 位置(label)ごとに除いて選び直し、除いた位置で検知漏れ・誤検知を数える(leave-one-group-out)。
"""
from dataclasses import dataclass


@dataclass
class Selection:
    threshold: int
    miss: float          # 検知漏れ率(パレットありで点数 < しきい値)
    false: float         # 誤検知率(パレットなしで点数 >= しきい値)
    margin: int          # しきい値の両側で、最も近い記録どうしの点数の差(分離の幅。大きいほど安定。分離できなければ 0)
    separable: bool      # パレットなしの最大 < パレットありの最小


def _errors(pos, neg, thr):
    miss = sum(h < thr for h in pos) / len(pos)
    false = sum(h >= thr for h in neg) / len(neg)
    return miss, false


def select_threshold(pos, neg):
    """pos / neg: 帯内の点数の一覧(どちらも空でないこと)。"""
    if not pos or not neg:
        raise ValueError('パレットありとなしの記録が両方必要です')
    cands = range(1, max(pos) + 2)
    errs = {t: sum(_errors(pos, neg, t)) for t in cands}
    best = min(errs.values())
    ties = [t for t in cands if abs(errs[t] - best) < 1e-12]
    thr = (ties[0] + ties[-1]) // 2      # 同率の範囲の中央(1 つならその値)
    miss, false = _errors(pos, neg, thr)
    below = [h for h in neg if h < thr]      # 正しく通す記録(しきい値より低い)
    above = [h for h in pos if h >= thr]     # 正しく検知する記録
    margin = (min(above) - max(below)) if below and above else 0
    return Selection(thr, miss, false, margin, max(neg) < min(pos))


def leave_one_group_out(records):
    """records: (group, is_positive, hits) の一覧。除いた群での検知漏れ・誤検知の件数を返す。"""
    groups = sorted({g for g, _, _ in records})
    miss = pos_n = false = neg_n = 0
    picks = []
    for g in groups:
        train = [(p, h) for gg, p, h in records if gg != g]
        pos = [h for p, h in train if p]
        neg = [h for p, h in train if not p]
        if not pos or not neg:
            continue
        thr = select_threshold(pos, neg).threshold
        picks.append(thr)
        for gg, p, h in records:
            if gg != g:
                continue
            if p:
                pos_n += 1
                miss += h < thr
            else:
                neg_n += 1
                false += h >= thr
    return dict(miss=miss, positives=pos_n, false=false, negatives=neg_n, picks=picks)
