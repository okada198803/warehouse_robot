"""動画の場面 4 用: ロボットが x >= しきい値(既定 2.0 m、南通路の西の入口)に来たら、南通路にパレットを置く。

    python video_place_pallet.py            # 待機 → 自動で配置して終了
    python video_place_pallet.py --x 2.3    # しきい値を変える

荷物の積載(南の荷物は x=1.1)の後、東へ向かったところで置くため、手で置くタイミングを合わせる必要がない。
"""
import argparse
import os
import subprocess
import threading

import roslibpy

DOCKER = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docker')
SRC = 'source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && '


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=2.0)
    ap.add_argument('--aisle', default='south_aisle')
    ap.add_argument('--port', type=int, default=9090)
    a = ap.parse_args()
    ros = roslibpy.Ros('localhost', a.port)
    ros.run()
    hit = threading.Event()
    topic = roslibpy.Topic(ros, '/odom', 'nav_msgs/msg/Odometry', throttle_rate=200, queue_length=1)
    topic.subscribe(lambda m: hit.set() if m['pose']['pose']['position']['x'] >= a.x else None)
    print(f'x >= {a.x} を待っています(Ctrl+C で中止)')
    hit.wait()
    topic.unsubscribe()
    subprocess.run(['docker', 'compose', 'exec', '-T', '-u', 'ubuntu', 'ros', 'bash', '-c',
                    SRC + f'ros2 run ai_nav obstacle place {a.aisle}'], cwd=DOCKER,
                   env={**os.environ, 'COMPOSE_FILE': 'compose.yaml'})
    ros.terminate()


if __name__ == '__main__':
    main()
