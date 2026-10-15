"""名前空間付きの RosLink(複数ロボット検証用)。ros_link.py(単体構成)は変更せず、継承して使う。"""
import roslibpy

from ros_link import RosLink


class NsLink(RosLink):

    def __init__(self, host='localhost', port=9090, ns=''):
        # ns: 名前空間(例 'r1')。トピックは /<ns>/cmd_vel, /<ns>/ai/plan, /<ns>/ai/status
        self.ns = ns
        p = f'/{ns}' if ns else ''
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run(timeout=10)
        self.cmd_vel = roslibpy.Topic(self.ros, p + '/cmd_vel', 'geometry_msgs/msg/Twist')
        self.plan = roslibpy.Topic(self.ros, p + '/ai/plan', 'std_msgs/msg/String')
        self.status = roslibpy.Topic(self.ros, p + '/ai/status', 'std_msgs/msg/String')
        self.cmd_vel.advertise()
        self.plan.advertise()

    def close(self):
        try:
            super().close()
        except AttributeError:
            pass   # roslibpy は 1 プロセスで複数接続を閉じると例外を出す(終了処理のみ。無害)
