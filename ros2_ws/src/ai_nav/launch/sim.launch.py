"""Gazebo とロボット、ROS 2 との橋渡し、rosbridge を起動する。

    ros2 launch ai_nav sim.launch.py                 # 倉庫ワールド(既定)
    ros2 launch ai_nav sim.launch.py world:=field    # 春ロボ2024フィールド
    ros2 launch ai_nav sim.launch.py gz_gui:=false   # Gazebo をサーバのみで起動(軽量)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ワールドごとの設定(sdf, 追加で置くモデル, ロボット初期姿勢 x, y, yaw)
WORLDS = {
    'warehouse': {
        'sdf': ('ai_nav', 'worlds', 'warehouse.sdf'),
        'extra_models': [],
        'spawn': (0.8, 0.8, 0.0),
    },
    # 複数台の C2(南通路の仕切りを外した幅 1.5 m)。認識層の学習データの生成(tools/gen_scan_dataset.py)でも使う
    'warehouse_wide': {
        'sdf': ('ai_nav', 'worlds', 'warehouse_wide.sdf'),
        'extra_models': [],
        'spawn': (0.8, 0.8, 0.0),
    },
    'field': {
        'sdf': ('nav_dev', 'models', 'worlds', 'nav_slam.sdf'),
        'extra_models': [('nav_dev', 'models', 'field', 'model.sdf')],
        'spawn': (0.45, 0.5, 1.57),
    },
}


def share(pkg, *parts):
    return os.path.join(get_package_share_directory(pkg), *parts)


def setup(context):
    world = LaunchConfiguration('world').perform(context)
    if world not in WORLDS:
        raise RuntimeError(f'world must be one of {list(WORLDS)}')
    cfg = WORLDS[world]
    gz_gui = LaunchConfiguration('gz_gui').perform(context) == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    rosbridge = LaunchConfiguration('rosbridge').perform(context) == 'true'
    model_path = share('nav_dev', 'models')

    actions = [
        SetEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH', '/opt/ros/humble/share:' + model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share('ros_ign_gazebo', 'launch', 'ign_gazebo.launch.py')),
            launch_arguments=[('ign_args', ('-r -v 2 ' + ('' if gz_gui else '-s ')
                                            + share(*cfg['sdf'])))]),
    ]
    for model in cfg['extra_models']:
        actions.append(Node(package='ros_ign_gazebo', executable='create', output='screen',
                            arguments=['-file', share(*model), '-allow_renaming', 'false']))

    x, y, yaw = cfg['spawn']
    actions.append(Node(
        package='ros_ign_gazebo', executable='create', output='screen',
        arguments=['-name', 'LidarRobo4', '-file', os.path.join(model_path, 'LidarRobo4', 'model.sdf'),
                   '-x', str(x), '-y', str(y), '-z', '0.075', '-Y', str(yaw)]))

    actions.append(Node(
        package='ros_ign_bridge', executable='parameter_bridge', output='screen',
        parameters=[{
            'config_file': share('ai_nav', 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': use_sim_time,
        }],
        remappings=[('/odom/tf', '/tf')]))

    # ロボット内部の固定座標変換(LidarRobo4/model.sdf の値と一致させる)
    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher', output='log',
        arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'base_link']))
    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher', output='log',
        arguments=['--x', '0.15', '--z', '0.06',
                   '--frame-id', 'base_link', '--child-frame-id', 'base_scan']))

    if rosbridge:
        # Mac 側 Python(roslibpy)から接続する WebSocket サーバ
        actions.append(Node(package='rosbridge_server', executable='rosbridge_websocket',
                            output='screen', parameters=[{'port': 9090, 'address': '0.0.0.0'}]))
        actions.append(Node(package='rosapi', executable='rosapi_node', output='log'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='warehouse | field'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gz_gui', default_value='true',
                              description='Gazebo の GUI を表示するか'),
        DeclareLaunchArgument('rosbridge', default_value='true'),
        OpaqueFunction(function=setup),
    ])
