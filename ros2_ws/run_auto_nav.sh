#!/bin/bash
# 自動検証用: Gazebo(GUI なし)+ Nav2 + plan_executor + rosbridge を、画面なし(RViz なし)で起動する。
# host/run_auto.sh から docker compose exec -d で呼ばれる。ログ: コンテナ内 /tmp/nav.log
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
echo $$ > /tmp/nav.pid
exec ros2 launch ai_nav ai_nav.launch.py gz_gui:=false rviz:=false "$@" > /tmp/nav.log 2>&1
