"""ジェスチャー → 速度指令の変換(ROS/カメラに依存しない純粋関数)。"""
from dataclasses import dataclass

MAX_LINEAR = 0.25    # m/s(Nav2 設定の上限 0.26 に合わせる)
MAX_ANGULAR = 1.0    # rad/s
DEAD_ZONE = 0.08     # 手の水平位置の不感帯(画面幅比)


@dataclass
class Command:
    linear: float
    angular: float
    label: str
    emergency_stop: bool = False


def steer_from_x(hand_x):
    """手の水平位置(鏡像表示の画面座標 0..1)から旋回速度を求める。左に手 → 左旋回(+)。"""
    dx = hand_x - 0.5
    if abs(dx) < DEAD_ZONE:
        return 0.0
    # 不感帯の外側を -1..1 に正規化
    sign = 1.0 if dx > 0 else -1.0
    norm = min((abs(dx) - DEAD_ZONE) / (0.5 - DEAD_ZONE), 1.0)
    return -sign * norm * MAX_ANGULAR


def gesture_to_command(gesture, hand_x, score, min_score=0.6):
    """認識結果を速度指令に変換する。

    Open_Palm   : 前進(手の左右位置で操舵)
    Pointing_Up : その場旋回(手の左右位置の方向へ)
    Victory     : ゆっくり後退
    Closed_Fist : 停止 + 自律移動の非常停止
    その他/手なし : 停止(デッドマン)
    """
    if gesture is None or score < min_score:
        return Command(0.0, 0.0, '手なし: 停止')
    if gesture == 'Closed_Fist':
        return Command(0.0, 0.0, '握りこぶし: 非常停止', emergency_stop=True)
    if gesture == 'Open_Palm':
        return Command(MAX_LINEAR, steer_from_x(hand_x), 'パー: 前進')
    if gesture == 'Pointing_Up':
        w = steer_from_x(hand_x)
        return Command(0.0, w, '指差し: 旋回' if w else '指差し: 中央(停止)')
    if gesture == 'Victory':
        return Command(-0.1, 0.0, 'チョキ: 後退')
    return Command(0.0, 0.0, f'{gesture}: 停止')


class Smoother:
    """指令の急変を抑える一次遅れフィルタ(停止は即時)。"""

    def __init__(self, alpha=0.4):
        self.alpha = alpha
        self.v = 0.0
        self.w = 0.0

    def update(self, cmd):
        if cmd.linear == 0.0 and cmd.angular == 0.0:
            self.v = self.w = 0.0
        else:
            self.v += self.alpha * (cmd.linear - self.v)
            self.w += self.alpha * (cmd.angular - self.w)
        return self.v, self.w
