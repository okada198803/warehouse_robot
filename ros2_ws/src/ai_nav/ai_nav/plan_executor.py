"""/ai/plan (JSON 文字列) を受け取り、地点辞書に従って Nav2 で順に移動するノード。

入力  /ai/plan   std_msgs/String  例: {"action": "goto", "targets": ["shipping"]}
                                       {"action": "patrol", "targets": ["a", "b"], "loops": 2}
                                       {"action": "stop"}
出力  /ai/status std_msgs/String  JSON
      state: idle / navigating / arrived / done / blocked / failed / canceled / stopped / rejected
      blocked のとき: target(到達できなかった地点), remaining(未到達の地点列),
                      near_aisle(停止位置に最も近い通路), pose([x, y])

立ち往生の検出: 一定時間(stuck_timeout)ロボットがほとんど動かなければ、
Nav2 の回復動作を待たずにゴールを取り消して blocked を報告する。
再計画は上位(LLM 側)が行う。
"""
import json
import math
import os

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from ai_nav.plan_logic import aisle_scan_hits, load_places, parse_plan, yaw_to_quaternion


class PlanExecutor(Node):

    def __init__(self):
        super().__init__('plan_executor')
        default_places = os.path.join(
            get_package_share_directory('ai_nav'), 'config', 'places_warehouse.yaml')
        self.declare_parameter('places_file', default_places)
        self.declare_parameter('stuck_timeout', 8.0)     # [s]
        self.declare_parameter('stuck_distance', 0.10)   # [m] この距離以上動けば進行中とみなす
        self.declare_parameter('near_aisle_radius', 1.0)  # [m] 通路間は約 1.4 m。広すぎると隣の通路を誤登録する
        # 向きを変えている間は立ち往生としない。ただし位置が動かない時間がこれを超えたら停止とみなす
        self.declare_parameter('max_no_progress', 24.0)  # [s]
        self.max_no_progress = self.get_parameter('max_no_progress').value
        # 立ち往生後、向きを変えずに来た方向へ直線で後退する距離 [m]
        self.declare_parameter('retreat_dist', 4.0)      # 最大後退距離 [m]
        self.declare_parameter('retreat_clear', 2.5)     # 停止した通路の中心からこの距離に出たら止める [m]
        self.retreat_dist = self.get_parameter('retreat_dist').value
        self.retreat_clear = self.get_parameter('retreat_clear').value
        # LiDAR による通路の確認(停止原因の分類と、口頭で解除された通路の事前確認)
        self.declare_parameter('scan_min_hits', 3)         # 帯に入る点がこの数以上なら障害物あり
        self.declare_parameter('aisle_half_length', 1.2)   # [m] 通路中心から前後に見る長さ
        self.declare_parameter('aisle_half_width', 0.3)    # [m] 通路中心線から左右に見る幅(壁を数えない)
        self.declare_parameter('precheck', True)           # 口頭解除された通路の事前確認
        self.aisle_half_length = self.get_parameter('aisle_half_length').value
        self.aisle_half_width = self.get_parameter('aisle_half_width').value
        self.verify = set()      # 事前確認する通路(計画の verify)
        self.scan = None         # 最新の LaserScan
        self.odom_yaw = None
        self.retreat = None      # 後退中の状態
        self.odom_xy = None      # オドメトリの位置 (x, y)
        self.yaw = None          # 直近の向き [rad]
        self.turn_t = 0.0        # 最後に向きが変わった時刻
        self.places = load_places(self.get_parameter('places_file').value)
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_distance = self.get_parameter('stuck_distance').value
        self.near_aisle_radius = self.get_parameter('near_aisle_radius').value
        self.get_logger().info(f'loaded {len(self.places)} places')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/ai/status', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/ai/plan', self.on_plan, 10)
        self.create_timer(1.0, self.watchdog)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.1, self.retreat_tick)

        self.queue = []          # 残りの地点 id(先頭が現在の目的地)
        self.total = 0
        self.goal_handle = None
        self.plan_id = 0         # 古い計画のコールバックを無視するための世代番号
        self.pose = None         # 最新の自己位置 (x, y)
        self.anchor = None       # 立ち往生判定の基準 (x, y, 時刻)
        self.last_feedback_pub = 0.0
        self.publish_status('idle', message='ready')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- 入力 ----------
    def on_plan(self, msg):
        try:
            action, targets = parse_plan(msg.data, self.places)
        except ValueError as e:
            self.get_logger().warn(f'rejected plan: {e}')
            self.publish_status('rejected', message=str(e))
            return

        self.cancel_current()
        if action == 'stop':
            self.queue = []
            self.cmd_pub.publish(Twist())   # 念のため停止指令
            self.publish_status('stopped', message='stop requested')
            return

        self.queue = targets
        self.total = len(targets)
        self.verify = self.read_verify(msg.data, targets)
        self.get_logger().info(f'new plan #{self.plan_id}: {targets} verify={sorted(self.verify)}')
        self.send_next()

    def read_verify(self, text, targets):
        """計画の verify(口頭で解除された通路)のうち、この計画に含まれるもの。"""
        try:
            names = json.loads(text).get('verify', [])
        except (ValueError, AttributeError):
            return set()
        return {n for n in names if n in targets and self.places[n].get('kind') == 'aisle'}

    # ---------- Nav2 への送信 ----------
    def send_next(self):
        if not self.queue:
            self.publish_status('done', message='all targets reached')
            return
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.publish_status('failed', message='navigate_to_pose server not available')
            self.queue = []
            return

        pid = self.queue[0]
        p = self.places[pid]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(p['x'])
        pose.pose.position.y = float(p['y'])
        yaw = float(p['yaw'])
        if len(self.queue) > 1:
            # 途中の地点は、次の地点へ向かう向きにする(通路の入口でその場回転しないため)
            nxt = self.places[self.queue[1]]
            yaw = math.atan2(nxt['y'] - p['y'], nxt['x'] - p['x'])
        qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose
        index = self.total - len(self.queue) + 1
        self.anchor = None
        self.publish_status('navigating', target=pid, index=index)

        plan_id = self.plan_id
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=lambda fb: self.on_feedback(fb, plan_id, pid, index))
        future.add_done_callback(lambda f: self.on_goal_response(f, plan_id, pid))

    def on_goal_response(self, future, plan_id, pid):
        handle = future.result()
        if plan_id != self.plan_id:
            # 受理待ちの間に停止・再計画された場合、遅れて受理されたゴールを取り消す
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.publish_status('failed', target=pid, message='goal rejected by Nav2')
            self.queue = []
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self.on_result(f, plan_id, pid))

    def on_feedback(self, fb, plan_id, pid, index):
        if plan_id != self.plan_id:
            return
        pos = fb.feedback.current_pose.pose.position
        self.pose = (pos.x, pos.y)
        t = self.now()
        if self.anchor is None or math.dist(self.pose, self.anchor[:2]) > self.stuck_distance:
            self.anchor = (pos.x, pos.y, t)
        # 向きが変わっている間は「回転中」とみなし、立ち往生の判定を保留する
        q = fb.feedback.current_pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        if self.yaw is None or abs(math.atan2(math.sin(yaw - self.yaw), math.cos(yaw - self.yaw))) > 0.2:
            self.turn_t = t
            self.yaw = yaw
        if t - self.last_feedback_pub >= 1.0:   # 1 Hz に間引く
            self.last_feedback_pub = t
            self.publish_status('navigating', target=pid, index=index,
                                distance_remaining=round(fb.feedback.distance_remaining, 2))

    def watchdog(self):
        """一定時間動けていなければ立ち往生とみなす。"""
        if self.goal_handle is None or self.anchor is None:
            return
        elapsed = self.now() - self.anchor[2]     # 位置が動かなかった時間
        turning = self.now() - self.turn_t < self.stuck_timeout
        if elapsed < self.stuck_timeout:
            return
        if turning and elapsed < self.max_no_progress:
            return   # その場回転中なので保留
        pid = self.queue[0] if self.queue else None
        remaining = list(self.queue)
        near = self.nearest_aisle()
        cause, hits = self.classify(near)
        self.get_logger().warn(f'stuck on the way to {pid} (near {near}, cause {cause}, hits {hits})')
        self.begin_blocked(pid, remaining, near, cause, hits, 'no progress; path seems blocked')

    def begin_blocked(self, pid, remaining, near, cause, hits, message):
        """ゴールを取り消し、後退してから blocked を報告する(原因 cause と LiDAR の証拠 hits 付き)。"""
        self.cancel_current()
        self.cmd_pub.publish(Twist())
        self.queue = []
        self.verify = set()
        if self.odom_xy is not None:
            # 向きを変えず、来た方向へ直線で後退してから blocked を報告する
            # near は停止した時点の通路(後退後の位置では判定しない)
            self.retreat = {'start': self.odom_xy, 't0': self.now(), 'pid': pid,
                            'remaining': remaining, 'near': near, 'cause': cause, 'hits': hits,
                            'message': message}
            self.get_logger().info(f'retreating straight back {self.retreat_dist} m')
            return
        self.publish_status('blocked', target=pid, remaining=remaining, near_aisle=near,
                            cause=cause, scan_hits=hits, message=message)

    # ---------- LiDAR による通路の確認 ----------
    def on_scan(self, msg):
        self.scan = msg
        if not (self.get_parameter('precheck').value and self.verify and self.queue and self.odom_xy is not None):
            return
        # 口頭で解除された通路は、走行前にパレットが残っていないか確かめる
        for aisle in list(self.verify):
            if aisle not in self.queue:
                self.verify.discard(aisle)
                continue
            hits = self.hits_in(aisle)
            if hits is not None and hits >= self.get_parameter('scan_min_hits').value:
                self.get_logger().warn(f'precheck: obstacle in {aisle} (hits {hits})')
                pid = self.queue[0]
                self.begin_blocked(pid, list(self.queue), aisle, 'obstacle', hits,
                                   'precheck: obstacle remains in the aisle that was reported clear')
                return

    def hits_in(self, aisle):
        """最新の scan で通路の帯に入る点の数。scan・自己位置がなければ None。"""
        if self.scan is None or self.odom_xy is None or self.odom_yaw is None:
            return None
        sc = self.scan
        return aisle_scan_hits(sc.ranges, sc.angle_min, sc.angle_increment, sc.range_min,
                               sc.range_max, self.odom_xy, self.odom_yaw, self.places[aisle],
                               self.aisle_half_length, self.aisle_half_width)

    def classify(self, near):
        """立ち往生の原因を LiDAR で裏づける。(原因, 点の数)を返す。

        obstacle    停止位置の近くの通路に、LiDAR で障害物を確認できた
        unconfirmed 通路の近くで止まったが、障害物を確認できない(経路追従の失敗など)
        unlocated   どの通路の近くでもない
        unknown     scan が得られず判定できない
        """
        if near is None:
            return 'unlocated', None
        hits = self.hits_in(near)
        if hits is None:
            return 'unknown', None
        return ('obstacle' if hits >= self.get_parameter('scan_min_hits').value else 'unconfirmed'), hits

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def retreat_tick(self):
        """後退中は一定速度で後ろへ進み、距離に達したら blocked を報告する。"""
        r = self.retreat
        if r is None:
            return
        moved = math.dist(self.odom_xy, r['start']) if self.odom_xy else 0.0
        # 通路の中心から十分離れたら止める(西ホールの壁に当たらないように、最大距離より優先)
        away = False
        if self.odom_xy and r['near'] in self.places:
            c = self.places[r['near']]
            away = math.dist(self.odom_xy, (c['x'], c['y'])) >= self.retreat_clear
        if away or moved >= self.retreat_dist or self.now() - r['t0'] > 15.0:
            self.retreat = None
            self.cmd_pub.publish(Twist())
            if self.odom_xy is not None:
                self.pose = self.odom_xy
            self.publish_status('blocked', target=r['pid'], remaining=r['remaining'],
                                near_aisle=r['near'], cause=r['cause'], scan_hits=r['hits'],
                                message=r['message'] + '; retreated to re-plan')
            return
        cmd = Twist()
        cmd.linear.x = -0.1   # 後退(ロボットの向きと逆方向)
        self.cmd_pub.publish(cmd)

    def nearest_aisle(self):
        if self.pose is None:
            return None
        # まだ到達していない通路(queue 内)だけを候補にする。通過済みの通路を誤登録しないため
        best, best_d = None, self.near_aisle_radius
        for pid in self.queue:
            p = self.places[pid]
            if p.get('kind') != 'aisle':
                continue
            d = math.dist(self.pose, (p['x'], p['y']))
            if d < best_d:
                best, best_d = pid, d
        return best

    def on_result(self, future, plan_id, pid):
        if plan_id != self.plan_id:
            return
        self.goal_handle = None
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status('arrived', target=pid)
            self.queue.pop(0)
            self.send_next()
        elif status == GoalStatus.STATUS_CANCELED:
            self.publish_status('canceled', target=pid)
        else:
            # Nav2 自身が諦めた場合も、停止位置に通路があれば blocked として扱う
            near = self.nearest_aisle()
            state = 'blocked' if near else 'failed'
            cause, hits = self.classify(near)
            remaining = list(self.queue)
            self.queue = []
            self.publish_status(state, target=pid, remaining=remaining, near_aisle=near,
                                cause=cause, scan_hits=hits, message=f'nav2 status {status}')

    def cancel_current(self):
        self.plan_id += 1   # 進行中のコールバックを無効化
        self.anchor = None
        self.retreat = None   # 後退中なら中止
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    # ---------- 出力 ----------
    def publish_status(self, state, target=None, index=None, **extra):
        body = {'state': state, 'total': self.total}
        if target is not None:
            body['target'] = target
            body['target_name'] = self.places[target]['name']
        if index is not None:
            body['index'] = index
        if self.pose is not None:
            body['pose'] = [round(self.pose[0], 2), round(self.pose[1], 2)]
        body.update(extra)
        self.status_pub.publish(String(data=json.dumps(body, ensure_ascii=False)))


def main():
    rclpy.init()
    node = PlanExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
