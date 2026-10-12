"""ロボット周囲の LiDAR 最小距離と、ローカルコストマップの値を表示する(診断用)。"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


class N(Node):
    def __init__(self):
        super().__init__('check_scan')
        self.scan = None
        self.cm = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: setattr(self, 'cm', m), qos)
        self.create_timer(0.5, self.tick)

    def tick(self):
        if self.scan is None or self.cm is None:
            return
        s = self.scan
        names = ['前', '左前', '左', '左後', '後', '右後', '右', '右前']
        print('LiDAR 最小距離 [m] (ロボット基準、各 45° 区間):')
        for k, nm in enumerate(names):
            c = k * math.pi / 4
            vals = []
            for i, r in enumerate(s.ranges):
                a = s.angle_min + i * s.angle_increment
                d = (a - c + math.pi) % (2 * math.pi) - math.pi
                if abs(d) <= math.pi / 8 and math.isfinite(r) and r > s.range_min:
                    vals.append(r)
            print(f'  {nm:3s}: {min(vals):.2f}' if vals else f'  {nm:3s}: なし')
        m = self.cm
        w, h, res = m.info.width, m.info.height, m.info.resolution
        n_lethal = sum(1 for v in m.data if v >= 99)
        print(f'ローカルコストマップ: {w}x{h} 、障害物(99以上)のセル数={n_lethal}')
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        cx, cy = ox + w * res / 2, oy + h * res / 2
        print(f'  中心(ロボット付近)=({cx:.2f},{cy:.2f}) 中心セル={m.data[(h // 2) * w + w // 2]}')
        raise SystemExit


rclpy.init()
try:
    rclpy.spin(N())
except SystemExit:
    pass
