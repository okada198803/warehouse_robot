"""倉庫ワールドに複数台のロボットを置き、台ごとに Nav2 と計画実行ノードを起動する(名前空間 r1, r2)。

    ros2 launch ai_nav multi.launch.py                    # 2 台、Gazebo は GUI なし(自動検証用)
    ros2 launch ai_nav multi.launch.py gz_gui:=true       # Gazebo の GUI あり
    ros2 launch ai_nav multi.launch.py world:=warehouse_wide   # C2: 通路を広げたワールド(別 SDF)

単体構成(sim.launch.py / ai_nav.launch.py)は変更しない。モデル・ブリッジ・Nav2 の設定は
ai_nav/multi_gen.py が元ファイルから /tmp/pai_multi/ に生成する。
各ロボットは自分の LiDAR で相手を障害物として見る。通信はしない(協調の仕組みは検証条件ごとに足す)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from ai_nav.multi_gen import (ROBOTS, bridge_yaml, namespaced_sdf, relative_nav2_params,
                              tf_remaps)

GEN = '/tmp/pai_multi'
TF_REMAP = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def share(pkg, *parts):
    return os.path.join(get_package_share_directory(pkg), *parts)


def setup(context):
    world = LaunchConfiguration('world').perform(context)
    gz_gui = LaunchConfiguration('gz_gui').perform(context) == 'true'
    n = int(LaunchConfiguration('robots').perform(context))
    names = list(ROBOTS)[:n]
    model_path = share('nav_dev', 'models')
    os.makedirs(GEN, exist_ok=True)

    with open(os.path.join(model_path, 'LidarRobo4', 'model.sdf'), encoding='utf-8') as f:
        base_sdf = f.read()
    with open(share('ai_nav', 'params', 'nav2_params.yaml'), encoding='utf-8') as f:
        params_text = relative_nav2_params(f.read())
    params_file = os.path.join(GEN, 'nav2_params_multi.yaml')
    bridge_file = os.path.join(GEN, 'bridge_multi.yaml')
    with open(params_file, 'w', encoding='utf-8') as f:
        f.write(params_text)
    with open(bridge_file, 'w', encoding='utf-8') as f:
        f.write(bridge_yaml(names))

    actions = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', '/opt/ros/humble/share:' + model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share('ros_ign_gazebo', 'launch', 'ign_gazebo.launch.py')),
            launch_arguments=[('ign_args', '-r -v 2 ' + ('' if gz_gui else '-s ')
                               + share('ai_nav', 'worlds', f'{world}.sdf'))]),
    ]
    for ns in names:
        sdf = os.path.join(GEN, f'LidarRobo4_{ns}.sdf')
        with open(sdf, 'w', encoding='utf-8') as f:
            f.write(namespaced_sdf(base_sdf, ns))
        x, y, yaw = ROBOTS[ns]
        actions.append(Node(
            package='ros_ign_gazebo', executable='create', output='screen',
            arguments=['-name', f'LidarRobo4_{ns}', '-file', sdf,
                       '-x', str(x), '-y', str(y), '-z', '0.075', '-Y', str(yaw)]))

    actions.append(Node(
        package='ros_ign_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_file, 'use_sim_time': True,
                     'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        remappings=tf_remaps(names)))

    delayed = []
    map_yaml = share('ai_nav', 'maps', f'{world}.yaml')
    places = share('ai_nav', 'config', f'places_{world}.yaml')   # warehouse_wide は車線の経由地点を含む
    for ns in names:
        # 座標変換(各名前空間の tf に出す)。map->odom は恒等(単体構成と同じ)
        for args in (['--frame-id', 'map', '--child-frame-id', 'odom'],
                     ['--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
                     ['--x', '0.15', '--z', '0.06', '--frame-id', 'base_link',
                      '--child-frame-id', 'base_scan']):
            actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                                namespace=ns, output='log', arguments=args, remappings=TF_REMAP))
        actions.append(Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            namespace=ns, output='screen',
            parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}]))
        actions.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map', namespace=ns, output='screen',
            parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}]))
        # navigation_launch.py の namespace 引数はパラメータの読み込み用で、ノードの名前空間は
        # 付かない(bringup_launch.py と同様に PushRosNamespace で包む必要がある)
        delayed.append(GroupAction([
            PushRosNamespace(ns),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
                launch_arguments={'namespace': ns, 'use_sim_time': 'true',
                                  'params_file': params_file, 'autostart': 'true'}.items())]))
        # plan_executor は絶対名(/ai/plan など)で購読するため、名前空間の中へ付け替える(コードは変更しない)
        delayed.append(Node(
            package='ai_nav', executable='plan_executor', namespace=ns, output='screen',
            parameters=[{'use_sim_time': True, 'places_file': places}],
            remappings=[('/ai/plan', 'ai/plan'), ('/ai/status', 'ai/status'), ('/cmd_vel', 'cmd_vel'),
                        ('/odom', 'odom'), ('/scan', 'scan')]))

    # 全ロボット共通の rosbridge(Mac 側 Python から /<ns>/ai/plan などに接続する)
    actions.append(Node(package='rosbridge_server', executable='rosbridge_websocket',
                        output='screen', parameters=[{'port': 9090, 'address': '0.0.0.0'}]))
    actions.append(Node(package='rosapi', executable='rosapi_node', output='log'))
    # Gazebo とブリッジの起動を待ってから Nav2 を立ち上げる(ロボットが増えるぶん長めに待つ)
    actions.append(TimerAction(period=10.0, actions=delayed))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='warehouse | warehouse_wide(C2。maps/ にも同名の地図が必要)'),
        DeclareLaunchArgument('gz_gui', default_value='false'),
        DeclareLaunchArgument('robots', default_value='2', description='ロボットの台数(2 または 3)'),
        OpaqueFunction(function=setup),
    ])
