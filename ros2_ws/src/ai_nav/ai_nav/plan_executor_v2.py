"""plan_executor の改良版(複数台の失敗の型 B・型 C への規則の修正)。元の plan_executor.py は変更しない。

パラメータ(既定は改良あり。元の動作に戻すには queue / 0 を指定)
  aisle_locate   'position'(既定): 停止位置がどの通路の範囲に入っているかで通路を特定する(型 C の修正)
                 'queue'         : 元の動作(未到達の通路だけを候補にする)
  abort_retry    Nav2 が中断(ABORTED)したとき、コストマップを消して同じ目的地を再送する回数(既定 1。型 B の修正)
                 0 で元の動作(中断で失敗とする)
  retreat_timeout 立ち往生の後に後退する時間の上限 [s](既定 40。元は 15 で固定)
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

from ai_nav.aisle_locate import locate_aisle, should_retry_abort
from ai_nav.plan_executor import PlanExecutor


class PlanExecutorV2(PlanExecutor):

    def __init__(self):
        super().__init__()
        self.declare_parameter('aisle_locate', 'position')
        self.declare_parameter('locate_half_length', 1.6)
        self.declare_parameter('locate_half_width', 1.1)
        self.declare_parameter('abort_retry', 1)
        self.declare_parameter('retreat_timeout', 40.0)
        self.abort_count = {}   # 目的地 id → 中断後に再送した回数(計画ごとに初期化)
        # 名前空間の中の相対名(複数台では /r1/global_costmap/... になる)
        self.clear_clients = [self.create_client(ClearEntireCostmap, n) for n in
                              ('global_costmap/clear_entirely_global_costmap',
                               'local_costmap/clear_entirely_local_costmap')]
        self.get_logger().info(
            f"v2: aisle_locate={self.get_parameter('aisle_locate').value} "
            f"abort_retry={self.get_parameter('abort_retry').value} "
            f"retreat_timeout={self.get_parameter('retreat_timeout').value}")

    def on_plan(self, msg):
        self.abort_count = {}
        super().on_plan(msg)

    def nearest_aisle(self):
        if self.get_parameter('aisle_locate').value != 'position':
            return super().nearest_aisle()
        return locate_aisle(self.pose, self.places,
                            self.get_parameter('locate_half_length').value,
                            self.get_parameter('locate_half_width').value)

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
        limit = self.get_parameter('retreat_timeout').value
        if away or moved >= self.retreat_dist or self.now() - r['t0'] > limit:
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
