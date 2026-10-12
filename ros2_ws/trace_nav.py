"""走行中の状態を 1 秒ごとに表示する(診断用)。位置・向き、速度指令、経路の先頭付近。"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


class N(Node):
    def __init__(self):
        super().__init__('trace_nav')
        self.odom = None
        self.cmd = None
        self.plan = None
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m), 10)
        self.create_subscription(Path, '/plan', lambda m: setattr(self, 'plan', m), 10)
        self.create_timer(1.0, self.tick)
        self.t0 = time.time()

    def tick(self):
        if self.odom is None:
            return
        p = self.odom.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c = self.cmd
        cmd = f'v={c.linear.x:+.2f} w={c.angular.z:+.2f}' if c else 'v=? w=?'
        pl = ''
        if self.plan and self.plan.poses:
            ps = self.plan.poses
            pick = [ps[min(i, len(ps) - 1)].pose.position for i in (0, len(ps) // 4, len(ps) // 2, len(ps) - 1)]
            pl = f' plan({len(ps)}点)=' + ' '.join(f'({a.x:.1f},{a.y:.1f})' for a in pick)
        print(f'{time.time() - self.t0:6.1f}s pos=({p.position.x:.2f},{p.position.y:.2f}) '
              f'yaw={math.degrees(yaw):+.0f} {cmd}{pl}', flush=True)


rclpy.init()
try:
    rclpy.spin(N())
except KeyboardInterrupt:
    pass
