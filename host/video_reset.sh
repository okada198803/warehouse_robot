#!/bin/bash
# 動画収録のテイク間リセット(単一ロボット。単体構成のコンテナ pai-ros で使う)。
#   ./video_reset.sh          パレットを撤去し、箱を初期位置へ、ロボットを入荷口 (0.8, 0.8) へ戻し、コストマップを消す
#   ./video_reset.sh south    上に加えて、南通路にパレットを置く(場面 5 の準備)
# 通行止めの記憶は llm_commander.py の中にあるため、コマンダーは Ctrl+C で止めて起動し直す。
cd "$(dirname "$0")/../docker" || exit 1
export COMPOSE_FILE=compose.yaml     # 単体構成のコンテナを使う(複数台用の設定が残っていても上書き)
docker compose exec -T -u ubuntu ros bash -s "${1:-}" <<'SH'
source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash
ros2 run ai_nav obstacle clear
ros2 topic pub --once /ai/cargo_cmd std_msgs/msg/String "{data: reset}" >/dev/null
for i in 1 2; do
  ign service -s /world/warehouse/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 3000 \
    --req 'name: "LidarRobo4" position {x: 0.8 y: 0.8 z: 0.075} orientation {w: 1.0}' >/dev/null
done
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null
if [ "$1" = "south" ]; then ros2 run ai_nav obstacle place south_aisle; fi
echo "リセット完了"
SH
