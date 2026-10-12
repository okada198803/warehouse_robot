#!/bin/bash
# シミュレーションだけを起動する(2-3, 2-4 用)。コンテナ内の noVNC ターミナルで実行する。
# 使い方: bash ~/ros2_ws/run_sim.sh
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 launch ai_nav sim.launch.py gz_gui:=false
