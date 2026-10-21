"""認識層(M8): LiDAR 1 周分を、通路の座標で見た点の分布(格子の点数)に変換する。学習と推論で共通。ROS に依存しない。

元の規則は「確認帯(前後 1.2 m・左右 0.3 m)に入る点の数 ≥ しきい値」だけで判定する。棚・壁の縁の点も同じ 1 点に数えるため、
障害物のない通路で 16 点(しきい値ちょうど)になり誤登録した(docs/perception/R2_結果.md)。
ここでは、確認帯より広い範囲の点の並び方(格子ごとの点数)とロボットの相対位置を特徴量にし、学習した分類器に判定させる。
"""
import json
import math
import re

LIDAR_OFFSET = 0.15   # LiDAR はロボット中心の 0.15 m 前(LidarRobo4/model.sdf)。元の規則はこの補正をしていない
GX = (-1.6, 1.6, 16)    # 通路方向 x の範囲と分割数(0.2 m)
GY = (-0.6, 0.6, 12)    # 通路と直交する y の範囲と分割数(0.1 m)
N_FEATURES = GX[2] * GY[2] + 4
REF_WIDTH = 0.9         # 正規化の基準の通路幅(学習データの狭い通路)


def aisle_width(aisle):
    """通路の幅 [m]。places の width があればそれ、なければ説明文の「幅0.9m」から読む。読めなければ基準の幅。"""
    if aisle.get('width'):
        return float(aisle['width'])
    m = re.search(r'幅\s*([0-9.]+)\s*m', aisle.get('description', ''))
    return float(m.group(1)) if m else REF_WIDTH


def features(ranges, angle_min, angle_increment, range_min, range_max, robot_xy, robot_yaw, aisle, norm=False):
    """点は LiDAR の位置(ロボット中心の LIDAR_OFFSET 前)から投影する。格子の点数(GX × GY)と、ロボットの通路に対する相対位置 (dx/1.6, dy/1.1, cos, sin) を返す。

    norm=True のときは、通路と直交する方向(y)を通路幅で正規化する(幅 w の通路を基準の幅 0.9 m に縮める)。
    学習にない通路幅で誤登録が増えたため(R1、幅 1.5 m で 11%)、壁や棚の縁が格子の同じ位置に入るようにする。
    """
    ky = REF_WIDTH / aisle_width(aisle) if norm else 1.0
    nx, ny = GX[2], GY[2]
    grid = [0.0] * (nx * ny)
    wx, wy = (GX[1] - GX[0]) / nx, (GY[1] - GY[0]) / ny
    sx = robot_xy[0] + LIDAR_OFFSET * math.cos(robot_yaw)
    sy = robot_xy[1] + LIDAR_OFFSET * math.sin(robot_yaw)
    for i, r in enumerate(ranges):
        if r is None or not (range_min <= r <= range_max) or math.isinf(r) or math.isnan(r):
            continue
        a = robot_yaw + angle_min + i * angle_increment
        lx = sx + r * math.cos(a) - aisle['x']
        ly = (sy + r * math.sin(a) - aisle['y']) * ky
        ix = int((lx - GX[0]) // wx)
        iy = int((ly - GY[0]) // wy)
        if 0 <= ix < nx and 0 <= iy < ny:
            grid[iy * nx + ix] += 1.0
    dx = (robot_xy[0] - aisle['x']) / 1.6
    dy = (robot_xy[1] - aisle['y']) * ky / 1.1
    return grid + [dx, dy, math.cos(robot_yaw), math.sin(robot_yaw)]


class Recognizer:
    """学習した 2 層の全結合ネット(重みは JSON)。依存は標準ライブラリのみ(コンテナに追加のパッケージを入れない)。"""

    def __init__(self, path):
        with open(path, encoding='utf-8') as f:
            m = json.load(f)
        self.mean, self.std = m['mean'], m['std']
        self.w1, self.b1, self.w2, self.b2 = m['w1'], m['b1'], m['w2'], m['b2']
        self.threshold = m.get('threshold', 0.5)
        self.norm = bool(m.get('norm', False))   # 学習時に通路幅で正規化したか(features の norm に渡す)

    def prob(self, feats):
        x = [(v - mu) / sd for v, mu, sd in zip(feats, self.mean, self.std)]
        h = [max(0.0, sum(wi * xi for wi, xi in zip(row, x)) + b) for row, b in zip(self.w1, self.b1)]
        z = sum(w * hi for w, hi in zip(self.w2, h)) + self.b2
        return 1.0 / (1.0 + math.exp(-max(min(z, 30.0), -30.0)))

    def is_obstacle(self, feats):
        p = self.prob(feats)
        return p >= self.threshold, p
