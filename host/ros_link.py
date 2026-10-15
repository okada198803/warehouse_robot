"""rosbridge(WebSocket)経由で Docker 内の ROS 2 と通信する薄いラッパー。"""
import json

import roslibpy


class RosLink:

    def __init__(self, host='localhost', port=9090, ns=''):
        # ns: 複数ロボット構成の名前空間(例 'r1')。空なら単体構成と同じトピック名
        self.ns = ns
        p = f'/{ns}' if ns else ''
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run(timeout=10)
        self.cmd_vel = roslibpy.Topic(self.ros, p + '/cmd_vel', 'geometry_msgs/msg/Twist')
        self.plan = roslibpy.Topic(self.ros, p + '/ai/plan', 'std_msgs/msg/String')
        self.status = roslibpy.Topic(self.ros, p + '/ai/status', 'std_msgs/msg/String')
        self.cmd_vel.advertise()
        self.plan.advertise()

    def send_twist(self, linear, angular):
        self.cmd_vel.publish(roslibpy.Message({
            'linear': {'x': float(linear), 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': float(angular)},
        }))

    def send_plan(self, plan):
        self.plan.publish(roslibpy.Message({'data': json.dumps(plan, ensure_ascii=False)}))

    def on_status(self, callback):
        """callback(dict) を /ai/status 受信時に呼ぶ。"""
        self.status.subscribe(lambda m: callback(json.loads(m['data'])))

    def close(self):
        for t in (self.cmd_vel, self.plan, self.status):
            try:
                t.unadvertise() if t is not self.status else t.unsubscribe()
            except Exception:
                pass
        try:
            self.ros.terminate()
        except AttributeError:
            pass   # roslibpy は 1 プロセスで複数接続を閉じると例外を出す(終了処理のみ。無害)
