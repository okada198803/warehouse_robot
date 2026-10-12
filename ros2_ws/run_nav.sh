#!/bin/bash
# シミュレーション + Nav2 + RViz を起動する(2-5 以降用)。コンテナ内の noVNC ターミナルで実行する。
# 使い方: bash ~/ros2_ws/run_nav.sh                 通常
#         bash ~/ros2_ws/run_nav.sh cargo:=true     荷物の積載を模擬(cargo_mode:=hide で箱が消える方式)
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 launch ai_nav ai_nav.launch.py gz_gui:=false "$@"
