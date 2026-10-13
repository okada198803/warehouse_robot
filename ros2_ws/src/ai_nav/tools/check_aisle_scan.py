"""各通路の帯に、現在の LiDAR の点が何個入っているかを表示する(D・E の事前確認用)。

    ./ros_exec.sh python3 /home/ubuntu/ros2_ws/src/ai_nav/tools/check_aisle_scan.py

パレットのある通路は scan_min_hits(既定 16)以上、ない通路は 0 付近になるはず。
"""
import math
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from ai_nav.plan_logic import aisle_scan_hits, load_places


class Check(Node):

    def __init__(self):
        super().__init__('check_aisle_scan')
        self.scan = None
        self.odom = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)


def main():
    places = load_places(os.path.join(get_package_share_directory('ai_nav'), 'config', 'places_warehouse.yaml'))
    rclpy.init()
    node = Check()
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.scan is not None and node.odom is not None:
            break
    if node.scan is None or node.odom is None:
        sys.exit('/scan または /odom を受信できません(run_nav.sh が起動しているか確認)')
    p = node.odom.pose.pose.position
    q = node.odom.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    sc = node.scan
    print(f'robot x={p.x:.2f} y={p.y:.2f} yaw={yaw:.2f}  scan points={len(sc.ranges)} range_max={sc.range_max}')
    for pid, pl in places.items():
        if pl.get('kind') == 'aisle':
            hits = aisle_scan_hits(sc.ranges, sc.angle_min, sc.angle_increment, sc.range_min,
                                   sc.range_max, (p.x, p.y), yaw, pl)
            print(f'  {pid:14s} hits={hits}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
