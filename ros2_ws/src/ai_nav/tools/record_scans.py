"""V7 用: パレットの有無 × ロボットの位置 で LiDAR の生データを記録する(コンテナ内で実行)。

    ./ros_exec.sh python3 /home/ubuntu/ros2_ws/src/ai_nav/tools/record_scans.py

事前: シミュレータ一式が起動していること(host/run_auto.sh が起動した状態でよい)。
条件: パレットなし / 南 / 中央 / 北 のいずれか 1 か所(4 通り)× 48 地点 = 192 回。約 10〜15 分。
結果: /home/ubuntu/ros2_ws/v7_scans.json(ホストでは ros2_ws/v7_scans.json)。
      解析は host/analyze_v7.py(しきい値・帯の幅・帯の長さを変えて検知率/誤検知率を計算)。
"""
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from ai_nav.plan_logic import load_places

OUT = '/home/ubuntu/ros2_ws/v7_scans.json'
CONDS = [None, 'south_aisle', 'middle_aisle', 'north_aisle']
AISLE_Y = {'south_aisle': 1.05, 'middle_aisle': 2.5, 'north_aisle': 3.95}


def poses():
    out = []
    for aisle, yc in AISLE_Y.items():
        for x in (1.0, 1.5, 2.0, 2.5):
            for dy in (0.0, 0.15, -0.15):
                out.append((f'{aisle}:w x={x} dy={dy:+.2f}', x, yc + dy, 0.0))
        for yaw in (0.2, -0.2):
            out.append((f'{aisle}:w x=1.5 yaw={yaw:+.1f}', 1.5, yc, yaw))
        for x in (5.5, 6.0):
            out.append((f'{aisle}:e x={x}', x, yc, math.pi))
    return out


class Rec(Node):

    def __init__(self):
        super().__init__('record_scans')
        self.scan = None
        self.scan_n = 0
        self.odom = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)

    def on_scan(self, m):
        self.scan = m
        self.scan_n += 1


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def set_pose(x, y, yaw):
    req = (f'name: "LidarRobo4" position {{x: {x} y: {y} z: 0.075}} '
           f'orientation {{z: {math.sin(yaw / 2):.5f} w: {math.cos(yaw / 2):.5f}}}')
    for _ in range(2):
        subprocess.run(['ign', 'service', '-s', '/world/warehouse/set_pose', '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', req],
                       capture_output=True)


def spin_for(node, sec):
    t_end = time.time() + sec
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    rclpy.init()
    node = Rec()
    spin_for(node, 2.0)
    if node.scan is None or node.odom is None:
        sys.exit('/scan または /odom を受信できません(シミュレータが起動しているか確認)')
    records = []
    plist = poses()
    total = len(CONDS) * len(plist)
    k = 0
    for cond in CONDS:
        subprocess.run(['ros2', 'run', 'ai_nav', 'obstacle', 'clear'], capture_output=True)
        if cond:
            subprocess.run(['ros2', 'run', 'ai_nav', 'obstacle', 'place', cond], check=True, capture_output=True)
        spin_for(node, 2.0)
        for label, x, y, yaw in plist:
            k += 1
            set_pose(x, y, yaw)
            spin_for(node, 1.2)
            n0 = node.scan_n
            while node.scan_n < n0 + 3:
                rclpy.spin_once(node, timeout_sec=0.1)
            p = node.odom.pose.pose.position
            sc = node.scan
            records.append({
                'cond': cond or 'none', 'label': label, 'target': [x, y, yaw],
                'robot': [round(p.x, 3), round(p.y, 3), round(yaw_of(node.odom.pose.pose.orientation), 4)],
                'angle_min': sc.angle_min, 'angle_increment': sc.angle_increment,
                'range_min': sc.range_min, 'range_max': sc.range_max,
                'ranges': [None if (math.isinf(r) or math.isnan(r)) else round(r, 3) for r in sc.ranges],
            })
            print(f'[{k}/{total}] {cond or "none"} {label}', flush=True)
        with open(OUT, 'w', encoding='utf-8') as f:     # 条件ごとに保存(途中で止まっても残る)
            json.dump(records, f)
    subprocess.run(['ros2', 'run', 'ai_nav', 'obstacle', 'clear'], capture_output=True)
    set_pose(0.8, 0.8, 0.0)
    print(f'保存: {OUT}({len(records)} 件)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
