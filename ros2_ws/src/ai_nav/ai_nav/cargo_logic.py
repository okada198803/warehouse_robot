"""荷物の積載・荷降ろしの模擬(ROS・Gazebo に依存しない部分)。

ロボットに把持機構はない。荷物置き場(kind: cargo)に到着したら「積載」、計画の最後に到着したら
「荷降ろし」とみなし、荷物の箱(見た目のみ)をロボットの上へ移し、目的地の前に降ろす。
"""
import math

CARRY_Z = 0.45        # 積載中の箱の中心高さ [m](ロボット上面の少し上)
FLOOR_Z = 0.15        # 床に置いた箱の中心高さ [m](箱の高さ 0.3 m の半分)
DROP_AHEAD = 0.55     # 荷降ろしの位置: 目的地の向きに進んだ距離 [m]
BOUNDS = (0.25, 6.75, 0.25, 4.75)   # 床の内側(x_min, x_max, y_min, y_max)
HIDDEN_Z = -5.0       # 箱を隠すとき(床の下)


def cargo_ids(places):
    return [pid for pid, p in places.items() if p.get('kind') == 'cargo']


def drop_xy(place, ahead=DROP_AHEAD):
    """目的地の前(向きの先)に荷物を降ろす位置。壁にめり込まないよう床の内側に収める。"""
    x = place['x'] + ahead * math.cos(place['yaw'])
    y = place['y'] + ahead * math.sin(place['yaw'])
    x = min(max(x, BOUNDS[0]), BOUNDS[1])
    y = min(max(y, BOUNDS[2]), BOUNDS[3])
    return x, y


def lerp(a, b, t):
    t = min(max(t, 0.0), 1.0)
    return tuple(p + (q - p) * t for p, q in zip(a, b))


class CargoTracker:
    """/ai/status の列から、積載・荷降ろしの時点を判定する。

    arrived(荷物地点)        → 積載。まだ積んでいなければ loaded
    done(積載後に別の地点へ到着) → 荷降ろし。unloaded(降ろした荷物 id, 降ろす地点 id)
    停止・失敗・通行止めでは何もしない(箱はロボットに載ったまま)。
    """

    def __init__(self, places):
        self.places = places
        self.cargo = set(cargo_ids(places))
        self.loaded = None
        self.dest = None        # 積載後に最後に到着した、荷物地点以外の地点

    def update(self, st):
        """状態 1 件を処理し、イベントの列 [('loaded', id) | ('unloaded', id, dest)] を返す。"""
        state, target = st.get('state'), st.get('target')
        events = []
        if state == 'arrived' and target in self.places:
            if target in self.cargo:
                if self.loaded is None:
                    self.loaded = target
                    self.dest = None
                    events.append(('loaded', target))
            elif self.loaded is not None:
                self.dest = target
        elif state == 'done' and self.loaded is not None and self.dest is not None:
            events.append(('unloaded', self.loaded, self.dest))
            self.loaded = None
            self.dest = None
        return events

    def reset(self):
        self.loaded = None
        self.dest = None
