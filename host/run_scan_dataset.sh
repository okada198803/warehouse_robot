#!/bin/bash
# 認識層(M8)の学習データを生成する。単一用コンテナ(pai-ros)でシミュレータだけを起動し、ロボットと障害物を自動配置して LiDAR を記録する。
#   cd ~/Desktop/pai_final/host && ./run_scan_dataset.sh
# 事前: Docker Desktop を起動。複数台用コンテナ(pai-ros-multi)は止める(このスクリプトが止める)。所要 約 1 時間(無人)。
# 出力: ../docs/perception/data/scan_<world>_s<seed>.jsonl(途中で止めても、同じコマンドで続きから生成する)
set -u
cd "$(dirname "$0")/../docker" || exit 1
SRC="source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash"
dexec() { docker compose -f compose.yaml exec -T -u ubuntu ros bash -c "$*" < /dev/null; }
docker compose -f compose.multi.yaml stop >/dev/null 2>&1
docker compose -f compose.yaml up -d || exit 1
mkdir -p ../docs/perception/data ../ros2_ws/perception_data
dexec "$SRC && cd ~/ros2_ws && colcon build --symlink-install --packages-select ai_nav > /tmp/build.log 2>&1" || { echo "ビルド失敗(コンテナ内 /tmp/build.log)"; exit 1; }
# (ワールド 種 件数)。100 件ごとにシミュレータを起動し直す(長く続けると遅くなるため。第 2 版の生成で、1 分あたり約 27 件が 6 件まで低下)
JOBS=""
# 第 4 欄: aisle = 通路の範囲内から(停止原因の判定)、hall = ホールから通路を見る(事前確認)。種 4〜8 はホール
# 用途: 学習 = 狭い通路の種 1・2・4・6・7、しきい値の決定 = 種 8、テスト = 種 3・5 と幅の広いワールド(学習・しきい値に使わない)
for spec in "warehouse 1 500 aisle" "warehouse 2 500 aisle" "warehouse 3 300 aisle" "warehouse 4 400 hall" "warehouse 5 200 hall" \
            "warehouse 6 400 hall" "warehouse 7 400 hall" "warehouse 8 300 hall" \
            "warehouse_wide 1 300 aisle" "warehouse_wide 2 300 aisle" "warehouse_wide 3 200 aisle"; do
  set -- $spec
  for n in $(seq 100 100 "$3"); do JOBS="$JOBS$1 $2 $n $4
"; done
done
printf "%s" "$JOBS" | while read -r W S N R; do
  OUT=perception_data/scan_${W}_s${S}.jsonl
  HAVE=$( [ -f "../ros2_ws/$OUT" ] && wc -l < "../ros2_ws/$OUT" | tr -d ' ' || echo 0 )
  [ "$HAVE" -ge "$N" ] && continue
  echo "[$(date +%T)] シミュレータを起動します(world=$W)"
  dexec "bash /home/ubuntu/ros2_ws/stop_nav.sh" >/dev/null 2>&1
  docker compose -f compose.yaml exec -T -d -u ubuntu ros setsid bash -c "$SRC && ros2 launch ai_nav sim.launch.py world:=$W gz_gui:=false rosbridge:=false > /tmp/sim.log 2>&1" < /dev/null
  for i in $(seq 1 30); do
    sleep 4
    dexec "$SRC && timeout 5 ros2 topic echo --once /scan --field angle_min" >/dev/null 2>&1 && break
  done
  echo "[$(date +%T)] $W 種 $S($R): $HAVE → $N 件"
  dexec "$SRC && cd ~/ros2_ws && python3 src/ai_nav/tools/gen_scan_dataset.py --world $W --n $N --seed $S --region $R --out $OUT" || { echo "生成に失敗"; exit 1; }
  cp ../ros2_ws/$OUT ../docs/perception/data/
done
dexec "bash /home/ubuntu/ros2_ws/stop_nav.sh" >/dev/null 2>&1
echo "[$(date +%T)] 完了: ../docs/perception/data/"
ls -la ../docs/perception/data/
