#!/bin/bash
# 起動中のシミュレーション一式(Gazebo サーバ・GUI、ブリッジ、Nav2、plan_executor、cargo_sim、rosbridge)を全て止める。
# 手動・自動のどちらで起動したものも対象。複数の起動が重なると /scan・/odom・Gazebo のサービスが混ざるため、起動し直す前に実行する。
if [ -f /tmp/nav.pid ]; then
  kill -INT -- -"$(cat /tmp/nav.pid)" 2>/dev/null
  sleep 5
  kill -9 -- -"$(cat /tmp/nav.pid)" 2>/dev/null
  rm -f /tmp/nav.pid
fi
pkill -9 -x ign
pkill -9 -x parameter_bridge
pkill -9 -f 'ign gazebo'
pkill -9 -f 'ros2 launch'
pkill -9 -f rosbridge_websocket
pkill -9 -f plan_executor
pkill -9 -f cargo_sim
pkill -9 -x ruby
pkill -9 -f nav2_
sleep 2
exit 0
