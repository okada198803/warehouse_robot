#!/bin/bash
# 動画収録のテイク間リセット(単一ロボット。単体構成のコンテナ pai-ros で使う)。
#   ./video_reset.sh          パレットを撤去し、箱を初期位置へ、ロボットを入荷口 (0.8, 0.8) へ戻し、コストマップを消す
#   ./video_reset.sh south    上に加えて、南通路にパレットを置く(場面 5 の準備)
#   ./video_reset.sh robotbox 上に加えて、南通路の中心に停止ロボットを模した箱(0.30 × 0.24 m)を置く(場面 8 の準備。P1 と同じ配置)
# 通行止めの記憶は llm_commander.py の中にあるため、コマンダーは Ctrl+C で止めて起動し直す。
cd "$(dirname "$0")/../docker" || exit 1
export COMPOSE_FILE=compose.yaml     # 単体構成のコンテナを使う(複数台用の設定が残っていても上書き)
docker compose exec -T -u ubuntu ros bash -s "${1:-}" <<'SH'
source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash
ros2 run ai_nav obstacle clear
ign service -s /world/warehouse/remove --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean --timeout 3000 \
  --req 'name: "p1_robot_box" type: MODEL' >/dev/null 2>&1
ros2 topic pub --once /ai/cargo_cmd std_msgs/msg/String "{data: reset}" >/dev/null
for i in 1 2; do
  ign service -s /world/warehouse/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 3000 \
    --req 'name: "LidarRobo4" position {x: 0.8 y: 0.8 z: 0.075} orientation {w: 1.0}' >/dev/null
done
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null
if [ "$1" = "south" ]; then ros2 run ai_nav obstacle place south_aisle; fi
if [ "$1" = "robotbox" ]; then
  echo '<sdf version="1.6"><model name="p1_robot_box"><static>true</static><link name="l"><collision name="c"><geometry><box><size>0.30 0.24 0.20</size></box></geometry></collision><visual name="v"><geometry><box><size>0.30 0.24 0.20</size></box></geometry><material><ambient>0.9 0.5 0.1 1</ambient><diffuse>0.9 0.5 0.1 1</diffuse></material></visual></link></model></sdf>' > /tmp/p1_robot_box.sdf
  ros2 run ros_ign_gazebo create -world warehouse -name p1_robot_box -file /tmp/p1_robot_box.sdf -x 3.5 -y 1.05 -z 0.1 >/dev/null 2>&1
fi
echo "リセット完了"
SH
