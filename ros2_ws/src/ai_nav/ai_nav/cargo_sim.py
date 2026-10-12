"""荷物の積載・荷降ろしを Gazebo の箱で模擬するノード(見た目のみ。衝突なし)。

購読  /ai/status (到着・完了)、/odom (ロボット位置)、/ai/cargo_cmd ("reset" で箱を初期位置へ)
発行  /ai/status に state=loaded / unloaded を追加(利用者への報告用)
動作  mode=follow: 積載中、箱がロボットの上に載って一緒に動く
      mode=hide  : 積載で箱が消え、荷降ろしで目的地の前に現れる
箱の移動は `ign service .../set_pose` を使う(約 5 Hz)。
"""
import json
import math
import os
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from ai_nav.cargo_logic import (CARRY_Z, FLOOR_Z, HIDDEN_Z, CargoTracker, cargo_ids,
                                drop_xy, lerp)
from ai_nav.plan_logic import load_places, yaw_to_quaternion


class PoseSetter:
    """最新の目標だけを保持し、別スレッドで set_pose を呼ぶ(呼び出しが遅くても詰まらない)。"""

    def __init__(self, world, log=None):
        self.world = world
        self.log = log or (lambda msg: None)
        self.calls = 0
        self.fails = 0
        self.latest = {}          # 箱の名前 -> (x, y, z, yaw)
        self.lock = threading.Lock()
        self.wake = threading.Event()
        threading.Thread(target=self.run, daemon=True).start()

    def set(self, name, pose):
        with self.lock:
            self.latest[name] = pose
        self.wake.set()

    def run(self):
        while True:
            self.wake.wait()
            self.wake.clear()
            with self.lock:
                items, self.latest = list(self.latest.items()), {}
            for name, (x, y, z, yaw) in items:
                self.call(name, x, y, z, yaw)

    def call(self, name, x, y, z, yaw):
        qz, qw = yaw_to_quaternion(yaw)
        req = (f'name: "{name}" position {{x: {x:.3f} y: {y:.3f} z: {z:.3f}}} '
               f'orientation {{z: {qz:.4f} w: {qw:.4f}}}')
        t0 = time.monotonic()
        try:
            r = subprocess.run(['ign', 'service', '-s', f'/world/{self.world}/set_pose',
                                '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
                                '--timeout', '500', '--req', req],
                               capture_output=True, text=True, timeout=10, check=False)
            ok = r.returncode == 0 and 'true' in r.stdout
            detail = (r.stdout + r.stderr).strip()[:120]
        except subprocess.TimeoutExpired:
            ok, detail = False, 'timeout'
        self.calls += 1
        if not ok:
            self.fails += 1
        # 最初の数回と、失敗は記録する(積載中に箱が動かない原因の切り分け用)
        if self.calls <= 3 or not ok:
            self.log(f'set_pose {name} -> ({x:.2f}, {y:.2f}, {z:.2f}) '
                     f'{"ok" if ok else "FAILED: " + detail} {time.monotonic() - t0:.2f}s')


class CargoSim(Node):

    def __init__(self):
        super().__init__('cargo_sim')
        default_places = os.path.join(
            get_package_share_directory('ai_nav'), 'config', 'places_warehouse.yaml')
        self.declare_parameter('places_file', default_places)
        self.declare_parameter('world', 'warehouse')
        self.declare_parameter('mode', 'follow')          # follow / hide
        self.declare_parameter('carry_z', CARRY_Z)
        self.declare_parameter('follow_hz', 5.0)
        self.declare_parameter('anim_sec', 1.2)           # 載せる・降ろす動きの時間
        self.places = load_places(self.get_parameter('places_file').value)
        self.tracker = CargoTracker(self.places)
        self.setter = PoseSetter(self.get_parameter('world').value, self.get_logger().info)
        self.mode = self.get_parameter('mode').value
        self.carry_z = self.get_parameter('carry_z').value
        self.anim_sec = self.get_parameter('anim_sec').value
        self.pose = None              # ロボットの (x, y, yaw)
        self.anim = None              # {'name', 'from', 'to', 't0'}: 箱の移動の途中
        self.last_log = 0.0
        self.home = {pid: (p['box']['x'], p['box']['y'], FLOOR_Z, 0.0)
                     for pid, p in self.places.items() if p.get('kind') == 'cargo'}
        self.pub = self.create_publisher(String, '/ai/status', 10)
        self.create_subscription(String, '/ai/status', self.on_status, 10)
        self.create_subscription(String, '/ai/cargo_cmd', self.on_cmd, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(1.0 / self.get_parameter('follow_hz').value, self.tick)
        self.get_logger().info(f'cargo_sim: {cargo_ids(self.places)} mode={self.mode}')

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def on_cmd(self, msg):
        if msg.data.strip() == 'reset':
            self.tracker.reset()
            self.anim = None
            for name, pose in self.home.items():
                self.setter.set(name, pose)
            self.get_logger().info('cargo reset')

    def on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        if st.get('state') in ('loaded', 'unloaded'):
            return   # 自分が出した状態は無視する
        for ev in self.tracker.update(st):
            if ev[0] == 'loaded':
                self.load(ev[1])
            else:
                self.unload(ev[1], ev[2])

    def carry_pose(self):
        x, y, yaw = self.pose
        return (x, y, self.carry_z, yaw)

    def load(self, cid):
        if self.pose is None:
            return
        if self.mode == 'hide':
            self.setter.set(cid, (0.0, 0.0, HIDDEN_Z, 0.0))
        else:
            self.anim = {'name': cid, 'from': self.home[cid], 't0': time.monotonic(), 'to': None}
        self.report('loaded', cid, f'{self.places[cid]["name"]}を積載しました')

    def unload(self, cid, dest):
        x, y = drop_xy(self.places[dest])
        end = (x, y, FLOOR_Z, self.places[dest]['yaw'])
        if self.mode == 'hide' or self.pose is None:
            self.setter.set(cid, end)
        else:
            self.anim = {'name': cid, 'from': self.carry_pose(), 'to': end, 't0': time.monotonic()}
        self.report('unloaded', cid, f'{self.places[dest]["name"]}で荷降ろししました')

    def report(self, state, cid, message):
        body = {'state': state, 'target': cid, 'target_name': self.places[cid]['name'],
                'message': message}
        self.pub.publish(String(data=json.dumps(body, ensure_ascii=False)))

    def tick(self):
        """箱の位置を更新する: 積載中はロボットに追従、載せる・降ろす間は補間する。"""
        if self.mode != 'follow':
            return
        a = self.anim
        if a is not None:
            t = (time.monotonic() - a['t0']) / self.anim_sec
            target = a['to'] if a['to'] is not None else (self.carry_pose() if self.pose else a['from'])
            self.setter.set(a['name'], lerp(a['from'], target, t))
            if t >= 1.0:
                self.anim = None
            return
        if self.tracker.loaded and self.pose is not None:
            self.setter.set(self.tracker.loaded, self.carry_pose())
            if time.monotonic() - self.last_log > 3.0:
                self.last_log = time.monotonic()
                x, y, z, _ = self.carry_pose()
                self.get_logger().info(f'carrying {self.tracker.loaded} at ({x:.2f}, {y:.2f}, {z:.2f}) '
                                       f'set_pose calls={self.setter.calls} fails={self.setter.fails}')


def main():
    rclpy.init()
    node = CargoSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
