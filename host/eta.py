"""到着時間の予測(ROS 不要)。経路長と停止回数から、計画を送ってから完了までの秒数を見積もる。

    予測 = 経路長 / 速度 + 停止回数 × 1 回あたりの追加時間 + 荷物の積載待ち

パラメータは、無人の測定(V8: 障害物なしの経路を複数回走らせた実測)から最小二乗で求める(fit)。
既定値は、既存の測定(V3 の撤去済み条件 22.3 s・5.6 m、V2 の迂回 31.9 s・7.5 m)から決めた初期値。
"""
from dataclasses import dataclass

LOAD_WAIT = 2.0   # 荷物地点での積載待ち[s](plan_executor の load_wait)


@dataclass
class EtaModel:
    sec_per_m: float = 3.2         # 巡航 約 0.31 m/s 相当(走行中の速度指令は約 0.26 m/s。加減速を含む)
    sec_per_stop: float = 2.5      # 停止(到着・回頭)1 回あたり
    base: float = 0.0

    def predict(self, length_m, n_stops, n_cargo=0):
        return self.base + self.sec_per_m * length_m + self.sec_per_stop * n_stops + LOAD_WAIT * n_cargo


def fit(samples):
    """samples: (経路長 m, 停止回数, 荷物地点数, 実測 s) の一覧。最小二乗で sec_per_m・sec_per_stop・base を求める。"""
    import numpy as np
    if len(samples) < 3:
        raise ValueError('3 件以上の実測が必要です')
    a = np.array([[length, stops, 1.0] for length, stops, _, _ in samples], dtype=float)
    y = np.array([t - LOAD_WAIT * cargo for _, _, cargo, t in samples], dtype=float)
    (m, s, b), *_ = np.linalg.lstsq(a, y, rcond=None)
    return EtaModel(float(m), float(s), float(b))


def errors(model, samples):
    """各実測の誤差(予測 − 実測, s)と、平均絶対誤差・平均絶対誤差率を返す。"""
    errs = [model.predict(length, stops, cargo) - t for length, stops, cargo, t in samples]
    mae = sum(abs(e) for e in errs) / len(errs)
    mape = sum(abs(e) / t for e, (_, _, _, t) in zip(errs, samples)) / len(errs)
    return errs, mae, mape


class EtaTracker:
    """送信した計画の予測を覚え、完了したら実測と比べる。再計画で送り直したら、残りの予測に置き換える。"""

    def __init__(self):
        self.predicted = None
        self.t_send = None

    def start(self, predicted, now):
        self.predicted, self.t_send = predicted, now

    def finish(self, now):
        """(予測, 実測, 誤差) を返す。計画を送っていなければ None。"""
        if self.predicted is None:
            return None
        measured = now - self.t_send
        res = (self.predicted, measured, self.predicted - measured)
        self.predicted = self.t_send = None
        return res
