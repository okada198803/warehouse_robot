"""複数ロボット用の設定を、単体用の元ファイルから生成する(ROS 非依存。テスト可能)。

元ファイル(単体構成)は変更しない。名前空間 r1, r2 … ごとに次を作る:
  - ロボットモデル SDF: Gazebo のトピック名に /<ns>/ を付ける
  - ブリッジ設定 YAML: /clock は共通、他は /<ns>/ 付き
  - Nav2 パラメータ: 絶対指定の /scan, /odom を相対名にする(名前空間の中で解決される)
"""
import math

# 名前空間ごとの初期姿勢 (x, y, yaw)。倉庫は西(入荷口)と東(出荷口)の向かい合わせ
ROBOTS = {
    'r1': (0.8, 0.8, 0.0),
    'r2': (6.2, 0.8, math.pi),
}

# model.sdf 内でトピック名を持つ箇所(元の文字列, 名前空間 ns を入れた文字列)
_SDF_TOPICS = [
    ('<topic>scan</topic>', '<topic>/{ns}/scan</topic>'),
    ('<odom_topic>/odom</odom_topic>', '<odom_topic>/{ns}/odom</odom_topic>'),
    ('<tf_topic>/odom/tf</tf_topic>', '<tf_topic>/{ns}/odom/tf</tf_topic>'),
    ('<topic>cmd_vel</topic>', '<topic>/{ns}/cmd_vel</topic>'),
    ('<topic>joint_states</topic>', '<topic>/{ns}/joint_states</topic>'),
]


def namespaced_sdf(text, ns):
    """ロボットモデルの SDF 文字列に名前空間を付ける。各箇所がちょうど 1 つあることを確認する。"""
    for old, new in _SDF_TOPICS:
        if text.count(old) != 1:
            raise ValueError(f'{old!r} が {text.count(old)} 個ある(1 個のはず)')
        text = text.replace(old, new.format(ns=ns))
    return text


def _entry(ros, gz, ros_type, gz_type, direction):
    return (f'- ros_topic_name: "{ros}"\n  gz_topic_name: "{gz}"\n  ros_type_name: "{ros_type}"\n'
            f'  gz_type_name: "{gz_type}"\n  direction: {direction}\n')


def bridge_yaml(namespaces):
    out = '---\n' + _entry('/clock', '/clock', 'rosgraph_msgs/msg/Clock',
                           'ignition.msgs.Clock', 'GZ_TO_ROS')
    for ns in namespaces:
        p = f'/{ns}'
        out += _entry(f'{p}/cmd_vel', f'{p}/cmd_vel', 'geometry_msgs/msg/Twist',
                      'ignition.msgs.Twist', 'ROS_TO_GZ')
        out += _entry(f'{p}/odom', f'{p}/odom', 'nav_msgs/msg/Odometry',
                      'ignition.msgs.Odometry', 'GZ_TO_ROS')
        out += _entry(f'{p}/odom/tf', f'{p}/odom/tf', 'tf2_msgs/msg/TFMessage',
                      'ignition.msgs.Pose_V', 'GZ_TO_ROS')
        out += _entry(f'{p}/scan', f'{p}/scan', 'sensor_msgs/msg/LaserScan',
                      'ignition.msgs.LaserScan', 'GZ_TO_ROS')
    return out


def tf_remaps(namespaces):
    """ブリッジが出す /<ns>/odom/tf を、各名前空間の tf に付け替える。"""
    return [(f'/{ns}/odom/tf', f'/{ns}/tf') for ns in namespaces]


def relative_nav2_params(text):
    """Nav2 パラメータの絶対トピック(/scan, /odom)を相対名にする。"""
    n_scan = text.count('topic: /scan')
    n_odom = text.count('odom_topic: /odom')
    if n_scan != 2 or n_odom != 1:
        raise ValueError(f'想定外の個数: /scan={n_scan}, /odom={n_odom}')
    return text.replace('topic: /scan', 'topic: scan').replace('odom_topic: /odom', 'odom_topic: odom')
