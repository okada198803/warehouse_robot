#!/bin/bash
# Gazebo の画面のカメラを切り替える(動画でロボットを判別しやすくする)。Mac の T2 から実行する。
#   ./video_camera.sh top            真上から。倉庫全体を画面いっぱいに(既定は z=7.5 → 2.8)
#   ./video_camera.sh top 3.5        高さを指定(小さいほど拡大)
#   ./video_camera.sh follow         ロボットを追従(既定のオフセット: 後方 1.6 m・上 1.8 m)
#   ./video_camera.sh follow 1.2 1.4 後方・上のオフセットを指定
#   ./video_camera.sh free           追従を解除
cd "$(dirname "$0")/../docker" || exit 1
export COMPOSE_FILE=compose.yaml
MODE="${1:-top}"; A="${2:-}"; B="${3:-}"
docker compose exec -T -u ubuntu ros bash -s "$MODE" "$A" "$B" <<'SH'
source /opt/ros/humble/setup.bash >/dev/null 2>&1
MODE=$1; A=$2; B=$3
case "$MODE" in
  top)
    Z=${A:-2.8}
    # 真上を向き、倉庫の長辺(x)を画面の横に合わせる(roll 0, pitch 90°, yaw 90°)
    Q=$(python3 -c "
import math
r,p,y=0,math.pi/2,math.pi/2
cr,sr,cp,sp,cy,sy=math.cos(r/2),math.sin(r/2),math.cos(p/2),math.sin(p/2),math.cos(y/2),math.sin(y/2)
print(f'x: {sr*cp*cy-cr*sp*sy:.5f} y: {cr*sp*cy+sr*cp*sy:.5f} z: {cr*cp*sy-sr*sp*cy:.5f} w: {cr*cp*cy+sr*sp*sy:.5f}')")
    ign service -s /gui/follow --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: ""' >/dev/null
    ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera --reptype ignition.msgs.Boolean --timeout 2000 \
      --req "pose: {position: {x: 3.5 y: 2.5 z: $Z} orientation: {$Q}}" ;;
  follow)
    DX=${A:-1.6}; DZ=${B:-1.8}
    ign service -s /gui/follow --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "LidarRobo4"'
    ign service -s /gui/follow/offset --reqtype ignition.msgs.Vector3d --reptype ignition.msgs.Boolean --timeout 2000 --req "x: -$DX y: 0 z: $DZ" ;;
  free)
    ign service -s /gui/follow --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: ""' ;;
  *) echo "使い方: video_camera.sh top|follow|free"; exit 1 ;;
esac
SH
