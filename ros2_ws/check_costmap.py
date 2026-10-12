"""グローバルコストマップの値を、指定地点で表示する(診断用)。"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

POINTS = {
    'cargo_north(1.1,3.4)': (1.1, 3.4), 'cargo_center(1.0,2.5)': (1.0, 2.5),
    'cargo_south(1.1,1.6)': (1.1, 1.6), 'hall(1.0,2.0)': (1.0, 2.0),
    'hall(1.0,1.0)': (1.0, 1.0), 'stuck(1.8,0.69)': (1.8, 0.69),
    'pallet(3.5,1.05)': (3.5, 1.05), 'middle_aisle(3.5,2.5)': (3.5, 2.5),
}


class N(Node):
    def __init__(self):
        super().__init__('check_costmap')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.cb, qos)

    def cb(self, m):
        res, ox, oy, w, h = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, m.info.width, m.info.height

        def cell(x, y):
            i, j = int((x - ox) / res), int((y - oy) / res)
            return m.data[j * w + i] if 0 <= i < w and 0 <= j < h else -1

        print(f'costmap {w}x{h} res={res}  (0=空き, 99前後=膨張, 100=障害物)')
        for k, (x, y) in POINTS.items():
            mx = max(cell(x + dx * res, y + dy * res) for dx in range(-3, 4) for dy in range(-3, 4))
            print(f'{k:24s} 中心={cell(x, y):4d}  周囲0.15 m 最大={mx:4d}')
        raise SystemExit


rclpy.init()
try:
    rclpy.spin(N())
except SystemExit:
    pass
