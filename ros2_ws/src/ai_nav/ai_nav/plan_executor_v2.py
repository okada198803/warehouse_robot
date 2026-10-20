"""plan_executor の改良版(複数台の失敗の型 B・型 C への規則の修正)。元の plan_executor.py は変更しない。

パラメータ(既定は改良あり。元の動作に戻すには queue / 0 を指定)
  aisle_locate   'position'(既定): 停止位置がどの通路の範囲に入っているかで通路を特定する(型 C の修正)
                 'queue'         : 元の動作(未到達の通路だけを候補にする)
  abort_retry    Nav2 が中断(ABORTED)したとき、コストマップを消して同じ目的地を再送する回数(既定 1。型 B の修正)
                 0 で元の動作(中断で失敗とする)
  recognizer     停止原因の判定と、口頭で解除された通路の事前確認に使う認識器。実行中に ros2 param set で切り替えられる
                 'rule'(既定): 確認帯の点数(≥ scan_min_hits)。点はロボット中心から投影する(元の規則)
                 'rule_offset' : 同じ規則で、点を LiDAR の位置(中心の 0.15 m 前)から投影する(取り付け位置の補正)
                 'learned'     : 学習した分類器(recognizer_file)。事前確認は precheck_consecutive 回続けて「あり」で確定
  precheck_consecutive  learned の事前確認で、障害物ありが何回続いたら確定するか(既定 3。1 回ごとの誤判定を抑えるため)
  rear_stop      後退中、ロボットの真後ろの帯(幅 0.4 m)で、後端から rear_stop [m] 以内に点があれば後退をやめる(既定 0.2。0 で無効)。
                 第 3 版で追加。延ばした後退が後方を見ずに壁・隅へ押し付け、動けなくなった(型 B、docs/perception/M9_後退の後方確認.md)
  retreat_timeout 通路を特定できた立ち往生の後に後退する時間の上限 [s](既定 40。元は 15 で固定。特定できない停止は 15 のまま)
                 元の 15 秒(0.1 m/s で 1.5 m)では、幅 0.9 m の通路の後半で止まったときに通路の外へ出られず、
                 通路の中で向きを変えようとして壁際で止まった(動作確認 C0 dead #1、docs/perception/動作確認.md)。
                 通路の中心から retreat_clear(2.5 m)離れるまで後退を続け、ホールで向きを変えさせる
起動: ros2 launch ai_nav multi.launch.py executor:=plan_executor_v2(または ai_nav.launch.py executor:=...)
"""
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ClearEntireCostmap

import os

from ament_index_python.packages import get_package_share_directory

from ai_nav.aisle_locate import locate_aisle, rear_clearance, should_retry_abort
from ai_nav.plan_logic import aisle_scan_hits
from ai_nav.scan_features import LIDAR_OFFSET, Recognizer, features
from ai_nav.plan_executor import PlanExecutor


class PlanExecutorV2(PlanExecutor):

    def __init__(self):
        super().__init__()
        self.declare_parameter('aisle_locate', 'position')
        self.declare_parameter('locate_half_length', 1.6)
        self.declare_parameter('locate_half_width', 1.1)
        self.declare_parameter('abort_retry', 1)
        self.declare_parameter('retreat_timeout', 40.0)
        self.declare_parameter('recognizer', 'rule')
        self.declare_parameter('recognizer_file', os.path.join(
            get_package_share_directory('ai_nav'), 'config', 'recognizer.json'))
        self.declare_parameter('precheck_consecutive', 3)
        # 後退の後方確認(M9): ロボットの真後ろの帯で、後端からこの距離より近くに点があれば後退をやめる [m]。0 で無効
        self.declare_parameter('rear_stop', 0.2)
        self._recog = None
        self.last_prob = None
        self.pre_streak = {}     # 事前確認: 通路 → 障害物ありが続いた回数(learned)
        self.abort_count = {}   # 目的地 id → 中断後に再送した回数(計画ごとに初期化)
        # 名前空間の中の相対名(複数台では /r1/global_costmap/... になる)
        self.clear_clients = [self.create_client(ClearEntireCostmap, n) for n in
                              ('global_costmap/clear_entirely_global_costmap',
                               'local_costmap/clear_entirely_local_costmap')]
        self.get_logger().info(
            f"v2: aisle_locate={self.get_parameter('aisle_locate').value} "
            f"abort_retry={self.get_parameter('abort_retry').value} "
            f"retreat_timeout={self.get_parameter('retreat_timeout').value} "
            f"recognizer={self.get_parameter('recognizer').value} (実行中に変更可) "
            f"rear_stop={self.get_parameter('rear_stop').value}")

    def nearest_aisle(self):
        if self.get_parameter('aisle_locate').value != 'position':
            return super().nearest_aisle()
        return locate_aisle(self.pose, self.places,
                            self.get_parameter('locate_half_length').value,
                            self.get_parameter('locate_half_width').value)

    def mode(self):
        return self.get_parameter('recognizer').value

    def recog(self):
        if self._recog is None:
            self._recog = Recognizer(self.get_parameter('recognizer_file').value)
        return self._recog

    def hits_in(self, aisle):
        """rule_offset のときは、点を LiDAR の位置から投影する(元は中心から)。"""
        if self.mode() != 'rule_offset':
            return super().hits_in(aisle)
        if self.scan is None or self.odom_xy is None or self.odom_yaw is None:
            return None
        sc, (x, y), yaw = self.scan, self.odom_xy, self.odom_yaw
        sxy = (x + LIDAR_OFFSET * math.cos(yaw), y + LIDAR_OFFSET * math.sin(yaw))
        return aisle_scan_hits(sc.ranges, sc.angle_min, sc.angle_increment, sc.range_min, sc.range_max,
                               sxy, yaw, self.places[aisle], self.aisle_half_length, self.aisle_half_width)

    def learned_prob(self, aisle):
        sc = self.scan
        f = features(sc.ranges, sc.angle_min, sc.angle_increment, sc.range_min, sc.range_max,
                     self.odom_xy, self.odom_yaw, self.places[aisle])
        return self.recog().is_obstacle(f)

    def classify(self, near):
        """停止原因の判定。learned のときは学習した分類器で判定し、確認帯の点数(hits)は記録のために残す。"""
        self.last_prob = None
        if self.mode() != 'learned' or near is None:
            return super().classify(near)
        hits = self.hits_in(near)
        if hits is None:
            return 'unknown', None
        ok, p = self.learned_prob(near)
        self.last_prob = round(p, 3)
        self.get_logger().info(f'recognizer: {near} p={p:.3f} (rule hits {hits})')
        return ('obstacle' if ok else 'unconfirmed'), hits

    def on_plan(self, msg):
        self.pre_streak = {}
        self.abort_count = {}
        PlanExecutor.on_plan(self, msg)

    def on_scan(self, msg):
        """事前確認。rule / rule_offset は元の処理(hits_in を通るので rule_offset も反映)。learned は分類器で判定する。"""
        if self.mode() != 'learned':
            return super().on_scan(msg)
        self.scan = msg
        if not (self.get_parameter('precheck').value and self.verify and self.queue and self.odom_xy is not None
                and self.odom_yaw is not None):
            return
        need = self.get_parameter('precheck_consecutive').value
        for aisle in list(self.verify):
            if aisle not in self.queue:
                self.verify.discard(aisle)
                continue
            ok, p = self.learned_prob(aisle)
            self.pre_streak[aisle] = self.pre_streak.get(aisle, 0) + 1 if ok else 0
            if self.pre_streak[aisle] >= need:
                hits = super().hits_in(aisle)
                self.last_prob = round(p, 3)
                self.get_logger().warn(f'precheck (learned): obstacle in {aisle} (p {p:.3f}, rule hits {hits})')
                pid = self.queue[0]
                self.begin_blocked(pid, list(self.queue), aisle, 'obstacle', hits,
                                   'precheck: obstacle remains in the aisle that was reported clear')
                return

    def publish_status(self, state, target=None, index=None, **extra):
        if state == 'blocked' and self.last_prob is not None and 'recog_prob' not in extra:
            extra['recog_prob'] = self.last_prob
        super().publish_status(state, target, index, **extra)

    def retreat_tick(self):
        """元の retreat_tick と同じ。後退の時間の上限だけをパラメータにした(元は 15 秒で固定)。"""
        r = self.retreat
        if r is None:
            return
        moved = math.dist(self.odom_xy, r['start']) if self.odom_xy else 0.0
        away = False
        if self.odom_xy and r['near'] in self.places:
            c = self.places[r['near']]
            away = math.dist(self.odom_xy, (c['x'], c['y'])) >= self.retreat_clear
        # 通路を特定できた停止だけ、通路の外へ出るまで長く後退する。特定できない停止(ホールなど)は元の 15 秒
        limit = self.get_parameter('retreat_timeout').value if r['near'] in self.places else 15.0
        rear_block = False
        stop = self.get_parameter('rear_stop').value
        if stop > 0 and self.scan is not None:
            sc = self.scan
            rear_block = rear_clearance(sc.ranges, sc.angle_min, sc.angle_increment, sc.range_min, sc.range_max) < stop
            if rear_block:
                self.get_logger().info('retreat: obstacle behind, stop retreating')
        if away or rear_block or moved >= self.retreat_dist or self.now() - r['t0'] > limit:
            self.retreat = None
            self.cmd_pub.publish(Twist())
            if self.odom_xy is not None:
                self.pose = self.odom_xy
            self.publish_status('blocked', target=r['pid'], remaining=r['remaining'],
                                near_aisle=r['near'], cause=r['cause'], scan_hits=r['hits'],
                                message=r['message'] + '; retreated to re-plan')
            return
        cmd = Twist()
        cmd.linear.x = -0.1
        self.cmd_pub.publish(cmd)

    def on_result(self, future, plan_id, pid):
        if plan_id == self.plan_id:
            status = future.result().status
            n = self.abort_count.get(pid, 0)
            if status == GoalStatus.STATUS_ABORTED and \
                    should_retry_abort(n, self.get_parameter('abort_retry').value):
                self.goal_handle = None
                self.abort_count[pid] = n + 1
                self.get_logger().warn(f'nav2 aborted on the way to {pid}: clear costmaps and retry ({n + 1})')
                self.publish_status('retry', target=pid, message=f'nav2 status {status}; cleared costmaps and retried')
                for c in self.clear_clients:
                    if c.service_is_ready():
                        c.call_async(ClearEntireCostmap.Request())
                self.anchor = None
                self.start_after(1.0, plan_id)   # 消去を待ってから同じ目的地を再送する
                return
        super().on_result(future, plan_id, pid)


def main():
    rclpy.init()
    node = PlanExecutorV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
