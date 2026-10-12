#!/bin/bash
# 自動検証用: 起動中のシミュレーション一式を止める(手動で起動したものも含む)。
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
pkill -9 -f nav2_
sleep 2
exit 0
