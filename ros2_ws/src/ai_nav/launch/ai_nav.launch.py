"""シミュレーション + Nav2 + 計画実行ノード。

    ros2 launch ai_nav ai_nav.launch.py                  # 倉庫ワールド、Gazebo GUI + RViz
    ros2 launch ai_nav ai_nav.launch.py gz_gui:=false    # 軽量(RViz のみ)
    ros2 launch ai_nav ai_nav.launch.py world:=field     # 春ロボ2024フィールド

自己位置推定: シミュレータのオドメトリ(/odom)は世界座標そのもので、地図も同じ原点で
作られている(倉庫は壁定義から生成、フィールドは同条件の SLAM 結果)ため、
map->odom は恒等変換で固定する(AMCL は使わない)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory('ai_nav')
    world = LaunchConfiguration('world').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    rviz = LaunchConfiguration('rviz').perform(context) == 'true'
    params_file = LaunchConfiguration('params_file').perform(context)
    map_yaml = os.path.join(share, 'maps', f'{world}.yaml')
    places = os.path.join(share, 'config', f'places_{world}.yaml')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': world,
            'use_sim_time': str(use_sim_time).lower(),
            'gz_gui': LaunchConfiguration('gz_gui').perform(context),
        }.items())

    map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', output='log',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'])

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}])
    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['map_server']}])

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': str(use_sim_time).lower(),
            'params_file': params_file,
            'autostart': 'true',
        }.items())

    executor = Node(
        package='ai_nav', executable='plan_executor', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'places_file': places}])

    delayed = [navigation, executor]
    if LaunchConfiguration('cargo').perform(context) == 'true':
        # 荷物の積載・荷降ろしの模擬(見た目のみ)。mode: follow(箱が載って移動)/ hide(消えて現れる)
        delayed.append(Node(
            package='ai_nav', executable='cargo_sim', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'places_file': places,
                         'world': world, 'mode': LaunchConfiguration('cargo_mode').perform(context)}]))
    if rviz:
        delayed.append(Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', os.path.join(share, 'config', 'ai_nav.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]))

    return [
        sim,
        map_to_odom,
        map_server,
        map_lifecycle,
        # Gazebo とブリッジ(/clock, /scan, TF)の起動を待ってから Nav2 を立ち上げる
        TimerAction(period=8.0, actions=delayed),
    ]


def generate_launch_description():
    share = get_package_share_directory('ai_nav')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse', description='warehouse | field'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gz_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('cargo', default_value='false', description='荷物の積載を模擬する'),
        DeclareLaunchArgument('cargo_mode', default_value='follow', description='follow | hide'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'params', 'nav2_params.yaml')),
        OpaqueFunction(function=setup),
    ])
