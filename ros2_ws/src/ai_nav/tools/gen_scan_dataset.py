"""認識層(M8)の学習データを Gazebo で自動生成する。コンテナ内で、シミュレータ(sim.launch.py)を起動した状態で実行する。

    python3 ~/ros2_ws/src/ai_nav/tools/gen_scan_dataset.py --world warehouse --n 600 --seed 1 \
        --out ~/ros2_ws/../docs/perception/data/scan_warehouse_s1.jsonl

1 件 = ロボットを通路の範囲内の位置・向きに瞬間移動し、障害物(なし / パレット / 停止ロボットを模した箱)を置いて、
LiDAR 1 周分を記録する。正解ラベルは「その通路の確認帯(中心から前後 1.2 m、左右 0.3 m。plan_executor と同じ)に
障害物が入っているか」で、シミュレータが置いた位置から自動で付ける(手作業のラベル付けはしない)。
比較のため、元の規則(帯に入る点の数)も同時に記録する。
"""
import argparse
import importlib.util
import json
import math
import os
import random
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

from ai_nav.plan_logic import aisle_scan_hits, load_places

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
GW = 'warehouse'     # SDF のワールド名(warehouse_wide・warehouse_mid も同じ名前)
ROBOT = 'LidarRobo4'
R_ROBOT = 0.22       # 瞬間移動の位置が壁・棚に重ならないための半径 [m]
BAND_HL, BAND_HW = 1.2, 0.3
# 障害物の種類: (名前, x 方向の大きさ, y 方向の大きさ, 高さ)
OBJECTS = {'pallet': (0.5, 0.7, 0.45), 'robot': (0.30, 0.24, 0.20)}
PARK = (-6.0, -6.0)  # 使わない障害物の待避場所(壁の外)


def boxes(world):
    spec = importlib.util.spec_from_file_location('gw', os.path.join(HERE, 'gen_warehouse.py'))
    gw = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gw)
    b = gw.BOXES
    if world == 'warehouse_wide':
        b = [x for x in b if x[0] != 'gate_south']
    elif world == 'warehouse_mid':
        b = [x if x[0] != 'gate_south' else ('gate_south', 3.5, 0.15, 0.2, 0.3, x[5], x[6]) for x in b]
    return [(cx, cy, sx, sy) for _n, cx, cy, sx, sy, _z, _c in b]


def circle_hits_box(x, y, r, box):
    cx, cy, sx, sy = box
    dx = max(abs(x - cx) - sx / 2, 0.0)
    dy = max(abs(y - cy) - sy / 2, 0.0)
    return dx * dx + dy * dy < r * r


def rect_overlap(ax, ay, asx, asy, b):
    cx, cy, sx, sy = b
    return abs(ax - cx) < (asx + sx) / 2 and abs(ay - cy) < (asy + sy) / 2


def in_band(obj, aisle):
    """障害物(中心 x, y、大きさ sx, sy)が通路の確認帯に重なるか。"""
    _t, x, y, sx, sy = obj
    return abs(x - aisle['x']) <= BAND_HL + sx / 2 and abs(y - aisle['y']) <= BAND_HW + sy / 2


def box_sdf(name, sx, sy, sz):
    return (f'<sdf version="1.6"><model name="{name}"><static>true</static><link name="link">'
            f'<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>'
            f'<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></visual>'
            f'</link></model></sdf>')


def ign_set_pose(name, x, y, z, yaw=0.0):
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    req = f'name: "{name}" position {{x: {x} y: {y} z: {z}}} orientation {{z: {qz} w: {qw}}}'
    r = subprocess.run(['ign', 'service', '-s', f'/world/{GW}/set_pose', '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', req],
                       capture_output=True, text=True)
    return 'true' in r.stdout


def spawn(name, sx, sy, sz):
    path = f'/tmp/{name}.sdf'
    with open(path, 'w') as f:
        f.write(box_sdf(name, sx, sy, sz))
    subprocess.run(['ros2', 'run', 'ros_ign_gazebo', 'create', '-world', GW, '-name', name, '-file', path,
                    '-x', str(PARK[0]), '-y', str(PARK[1]), '-z', str(sz / 2)], check=True,
                   capture_output=True)


class ScanTap(Node):
    def __init__(self):
        super().__init__('scan_dataset')
        self.msg, self.sim = None, 0.0
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)

    def cb(self, m):
        self.msg = m

    def on_clock(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9

    def sim_now(self, timeout=3.0):
        t0 = time.time()
        while self.sim == 0.0 and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.sim

    def after(self, t_sim, timeout=10.0):
        """シミュレータ時刻 t_sim より後に取得したスキャンを返す。
        第 1 版はスキャンの到着回数で待ったため、瞬間移動の前のスキャン(1 件前の位置)を記録していた(data/bad_lag/)。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            m = self.msg
            if m is not None and m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 >= t_sim:
                return m
        return None


def ray_box_dist(x, y, a, bx, rmax=20.0):
    """静的な壁・棚だけの地図で、(x, y) から角度 a の方向に最初に当たる距離。"""
    best, dx, dy = rmax, math.cos(a), math.sin(a)
    for cx, cy, sx, sy in bx:
        tmin, tmax, ok = 0.0, rmax, True
        for o, d, lo, hi in ((x, dx, cx - sx / 2, cx + sx / 2), (y, dy, cy - sy / 2, cy + sy / 2)):
            if abs(d) < 1e-9:
                ok = ok and lo <= o <= hi
            else:
                t1, t2 = (lo - o) / d, (hi - o) / d
                tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
        if ok and tmin <= tmax and tmin < best:
            best = tmin
    return best


def static_mismatch(m, x, y, yaw, bx):
    """スキャンと、記録した位置から計算した静的な地図の距離との差(中央値)。位置とスキャンの対応の確認に使う。
    障害物のある方向は大きくずれるが、中央値なので影響は小さい。"""
    sxp, syp = x + 0.15 * math.cos(yaw), y + 0.15 * math.sin(yaw)    # LiDAR はロボット中心の 0.15 m 前
    errs = []
    for i in range(0, len(m.ranges), 6):
        r = m.ranges[i]
        if not math.isfinite(r):
            continue
        errs.append(abs(min(r, 20.0) - ray_box_dist(sxp, syp, yaw + m.angle_min + i * m.angle_increment, bx)))
    errs.sort()
    return errs[len(errs) // 2] if errs else 99.0


def sample_pose(rng, aisles, places, bx, region='aisle'):
    """region=aisle: 通路の範囲内(停止原因の判定の状況)。region=hall: 通路の外のホール(事前確認の状況。
    口頭で解除された通路を、走る前に入荷口などから見る)。"""
    for _ in range(500):
        a = rng.choice(aisles)
        p = places[a]
        if region == 'hall':
            x = rng.choice([rng.uniform(0.4, 1.9), rng.uniform(5.1, 6.6)])
            y = rng.uniform(0.3, 4.7)
        else:
            x = p['x'] + rng.uniform(-1.6, 1.6)
            y = p['y'] + rng.uniform(-1.1, 1.1)
        if any(circle_hits_box(x, y, R_ROBOT, b) for b in bx):
            continue
        if not (0.25 < x < 6.75 and 0.25 < y < 4.75):
            continue
        if region == 'hall':   # 通路の方向を向く(事前確認は通路へ向かう途中に行う)
            yaw = math.atan2(p['y'] - y, p['x'] - x) + rng.gauss(0.0, 0.5)
        else:
            yaw = rng.choice([0.0, math.pi]) + rng.gauss(0.0, 0.5)
        return a, x, y, yaw
    raise RuntimeError('位置を選べません')


def sample_object(rng, aisle, robot_xy, bx):
    """通路の中かその近くに障害物を 1 つ置く。半分は確認帯の中、残りは帯の外(近く)も含めて一様に選ぶ。"""
    for _ in range(500):
        t = rng.choice(list(OBJECTS))
        sx, sy, _sz = OBJECTS[t]
        x = aisle['x'] + rng.uniform(-1.6, 1.6)
        y = aisle['y'] + rng.uniform(-0.35, 0.35)
        if math.dist((x, y), robot_xy) < 0.8:    # パレットの対角の半分 0.43 + ロボット 0.22 + 余裕。重なると瞬間移動でロボットが弾かれる
            continue
        if t == 'robot' and any(rect_overlap(x, y, sx, sy, b) for b in bx):
            continue
        return (t, x, y, sx, sy)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='warehouse', choices=['warehouse', 'warehouse_wide', 'warehouse_mid'])
    ap.add_argument('--n', type=int, default=600)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--p-obj', type=float, default=0.5, help='障害物を置く割合')
    ap.add_argument('--out', required=True)
    ap.add_argument('--region', default='aisle', choices=['aisle', 'hall'])
    args = ap.parse_args()

    places = load_places(os.path.join(PKG, 'config', f'places_{args.world}.yaml'))
    aisles = [k for k, v in places.items() if v.get('kind') == 'aisle']
    bx = boxes(args.world)
    rng = random.Random(args.seed)
    rclpy.init()
    tap = ScanTap()
    for name, (sx, sy, sz) in OBJECTS.items():
        spawn(f'ds_{name}', sx, sy, sz)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    done = sum(1 for _ in open(args.out)) if os.path.exists(args.out) else 0
    if done:   # 途中から再開する場合は、乱数の種を変えて残りを生成する(同じ配置を繰り返さないため)
        rng = random.Random(args.seed * 100003 + done)
    with open(args.out, 'a') as f:
        i = done
        while i < args.n:
            a, x, y, yaw = sample_pose(rng, aisles, places, bx, args.region)
            obj = sample_object(rng, places[a], (x, y), bx) if rng.random() < args.p_obj else None
            for name, (_sx, _sy, sz) in OBJECTS.items():
                if obj and obj[0] == name:
                    ign_set_pose(f'ds_{name}', obj[1], obj[2], sz / 2)
                else:
                    ign_set_pose(f'ds_{name}', PARK[0], PARK[1], sz / 2)
            if not ign_set_pose(ROBOT, x, y, 0.075, yaw):
                continue
            mis, m = 99.0, None
            for attempt in range(3):     # 合わなければ置き直す(倒れた・弾かれたロボットを立て直す)
                if attempt and not ign_set_pose(ROBOT, x, y, 0.075, yaw):
                    break
                t_moved = tap.sim_now()
                m = tap.after(t_moved + 0.6)     # 移動の完了後、0.6 秒(シミュレータ時刻)以降のスキャン
                if m is None:
                    break
                mis = static_mismatch(m, x, y, yaw, bx)
                if mis <= 0.15:
                    break
            if m is None:
                print('スキャンが来ません。シミュレータを確認してください')
                break
            if mis > 0.15:      # 3 回置いても位置とスキャンが合わない。記録せずに次の配置へ
                print(f'位置とスキャンが合いません(差の中央値 {mis:.2f} m)。この配置は記録しません')
                continue
            rec = {'mismatch': round(mis, 3), 'region': args.region, 'world': args.world, 'aisle': a, 'pose': [round(x, 3), round(y, 3), round(yaw, 4)],
                   'angle_min': m.angle_min, 'angle_increment': m.angle_increment,
                   'range_min': m.range_min, 'range_max': m.range_max,
                   'ranges': [round(r, 3) if math.isfinite(r) else None for r in m.ranges],
                   'obj': None if obj is None else {'type': obj[0], 'x': round(obj[1], 3), 'y': round(obj[2], 3)},
                   'label': int(obj is not None and in_band(obj, places[a])),
                   'rule_hits': aisle_scan_hits(m.ranges, m.angle_min, m.angle_increment, m.range_min,
                                                m.range_max, (x, y), yaw, places[a], BAND_HL, BAND_HW)}
            f.write(json.dumps(rec) + '\n')
            f.flush()
            i += 1
            if i % 50 == 0:
                print(f'{i}/{args.n}')
    for name, (_sx, _sy, sz) in OBJECTS.items():
        ign_set_pose(f'ds_{name}', PARK[0], PARK[1], sz / 2)
    tap.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
