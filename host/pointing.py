"""指差しの方向から荷物置き場を選ぶ(MediaPipe に依存しない部分。単体テスト可能)。

手のランドマーク(画像座標 21 点)から、人差し指だけを伸ばしているかを判定し、指す向きの
特徴量 u(左右方向、手の大きさで正規化)を求める。コントロールルームのカメラから見た
荷物置き場 3 方向について、操作者が 1 回ずつ指して u を記録(校正)し、以後は最も近い方向を選ぶ。
同じ方向を一定時間保持したら確定する。方向が荷物置き場の間(曖昧)、または範囲外なら選ばない。
"""
import json
import math
import os
import time

SELECTION_FILE = '/tmp/pai_pointing.json'
CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pointing_calib.json')
# 鏡像のカメラ映像で、操作者の左(北の荷物)が u<0、右(南の荷物)が u>0 になる想定の初期値。実機では校正で上書きする
DEFAULT_CALIB = {'cargo_north': -1.0, 'cargo_center': 0.0, 'cargo_south': 1.0}
AMBIGUOUS_RATIO = 0.40     # 最も近い方向までの距離 / (近い 2 方向までの距離の和)がこれ以上なら曖昧
OUT_OF_RANGE = 0.8         # 最も近い方向までの距離が、方向の間隔のこの倍を超えたら範囲外

# ランドマーク番号(MediaPipe Hands): 0 手首、5 人差し指の付け根、9 中指の付け根
WRIST, INDEX_MCP, MIDDLE_MCP = 0, 5, 9
FINGERS = {'index': (8, 6), 'middle': (12, 10), 'ring': (16, 14), 'pinky': (20, 18)}   # (指先, 第 2 関節)


def _d(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def hand_size(lm):
    return max(_d(lm[WRIST], lm[MIDDLE_MCP]), 1e-6)


def is_pointing(lm):
    """人差し指だけが伸び、他の 3 本が曲がっていれば True。"""
    wrist = lm[WRIST]
    tip, pip = (lm[i] for i in FINGERS['index'])
    if _d(tip, wrist) < _d(pip, wrist) * 1.15:
        return False
    for name in ('middle', 'ring', 'pinky'):
        tip, pip = (lm[i] for i in FINGERS[name])
        if _d(tip, wrist) > _d(pip, wrist) * 1.0:
            return False
    return True


def pointing_feature(lm):
    """指す向きの左右方向の特徴量 u: (人差し指の先 - 手首)の x を、手の大きさで割った値。"""
    return (lm[FINGERS['index'][0]].x - lm[WRIST].x) / hand_size(lm)


class Calibration:

    def __init__(self, centers=None):
        self.centers = dict(centers or DEFAULT_CALIB)

    @classmethod
    def fit(cls, samples):
        """samples: {cargo_id: [u, ...]} から、各方向の中央値を校正値にする。"""
        centers = {}
        for cid, vals in samples.items():
            v = sorted(vals)
            if v:
                centers[cid] = v[len(v) // 2]
        return cls(centers)

    def spacing(self):
        vals = sorted(self.centers.values())
        gaps = [b - a for a, b in zip(vals, vals[1:])]
        return min(gaps) if gaps else 1.0

    def classify(self, u):
        """(cargo_id | None, 状態)。状態は ok / ambiguous / out。"""
        ranked = sorted(self.centers.items(), key=lambda kv: abs(u - kv[1]))
        best_id, best_c = ranked[0]
        d1 = abs(u - best_c)
        if d1 > self.spacing() * OUT_OF_RANGE:
            return None, 'out'
        if len(ranked) > 1:
            d2 = abs(u - ranked[1][1])
            if d1 + d2 > 0 and d1 / (d1 + d2) >= AMBIGUOUS_RATIO:
                return None, 'ambiguous'
        return best_id, 'ok'

    def save(self, path=CALIB_FILE):
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(self.centers, f, ensure_ascii=False, indent=1)

    @classmethod
    def load(cls, path=CALIB_FILE):
        try:
            with open(path, encoding='utf-8') as f:
                data = json.load(f)
            if isinstance(data, dict) and data:
                return cls({k: float(v) for k, v in data.items()})
        except (OSError, ValueError):
            pass
        return cls()


class PointingSelector:
    """同じ方向を dwell 秒保持したら確定する。確定した荷物は ttl 秒だけ有効。"""

    def __init__(self, calib, dwell=1.0, ttl=10.0):
        self.calib = calib
        self.dwell = dwell
        self.ttl = ttl
        self.candidate = None
        self.since = None
        self.status = 'none'          # none / ok / ambiguous / out
        self.confirmed = None
        self.confirmed_at = None

    def update(self, u, now):
        """u: 指差し中の特徴量。指差しでなければ None。確定した瞬間だけ荷物 id を返す。"""
        if u is None:
            self.candidate, self.since, self.status = None, None, 'none'
            return None
        cid, self.status = self.calib.classify(u)
        if cid is None:
            self.candidate, self.since = None, None
            return None
        if cid != self.candidate:
            self.candidate, self.since = cid, now
            return None
        if now - self.since >= self.dwell and self.confirmed_at != self.since:
            self.confirmed, self.confirmed_at = cid, self.since
            return cid
        return None

    def progress(self, now):
        """確定までの進み具合 0..1(表示用)。"""
        if self.candidate is None or self.since is None:
            return 0.0
        return min((now - self.since) / self.dwell, 1.0)

    def current(self, now):
        if self.confirmed and self.confirmed_at is not None and \
                now - (self.confirmed_at + self.dwell) <= self.ttl:
            return self.confirmed
        return None


def write_selection(cargo_id, path=SELECTION_FILE, now=None):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump({'target': cargo_id, 't': time.time() if now is None else now}, f)


def read_selection(path=SELECTION_FILE, ttl=10.0, now=None):
    """確定済みの指差しが ttl 秒以内なら荷物 id、なければ None。"""
    try:
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
        age = (time.time() if now is None else now) - float(data['t'])
        if 0 <= age <= ttl and data.get('target'):
            return data['target']
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None


def clear_selection(path=SELECTION_FILE):
    try:
        os.remove(path)
    except OSError:
        pass


class PendingInstruction:
    """「どの荷物ですか?」と聞き返した指示を保持し、指差しが確定したら続行できるようにする。"""

    def __init__(self, ttl=30.0):
        self.ttl = ttl
        self.text = None
        self.t = 0.0

    def hold(self, text, now):
        self.text, self.t = text, now

    def cancel(self):
        self.text = None

    def expired(self, now):
        """保持中の指示が期限切れなら True(同時に破棄する)。"""
        if self.text is not None and now - self.t > self.ttl:
            self.text = None
            return True
        return False

    def take(self, selection, now):
        """指差し selection があり、保持中の指示が有効ならその指示を返して破棄する。"""
        if selection and self.text is not None and now - self.t <= self.ttl:
            text, self.text = self.text, None
            return text
        return None
