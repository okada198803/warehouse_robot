#!/bin/bash
# 複数ロボット検証用: 2 台(既定)の Gazebo + Nav2 + plan_executor + rosbridge を画面なしで起動する。
#   使い方(コンテナ内): bash ~/ros2_ws/run_multi.sh [world:=warehouse] [robots:=2] [gz_gui:=true]
# 止めるときは stop_nav.sh(名前空間付きのノードも対象)。ログ: コンテナ内 /tmp/nav.log
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
echo $$ > /tmp/nav.pid
exec ros2 launch ai_nav multi.launch.py "$@" > /tmp/nav.log 2>&1
