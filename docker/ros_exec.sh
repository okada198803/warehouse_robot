#!/bin/bash
# コンテナ内で ROS 2 のコマンドを実行する。
# 使い方: ./ros_exec.sh ros2 run ai_nav obstacle clear
cd "$(dirname "$0")" || exit 1
docker compose exec -u ubuntu ros bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && $*"
