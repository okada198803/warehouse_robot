#!/bin/bash
# 認識層(M8)の走行検証 P1 を無人で実行する(単一用コンテナ、plan_executor_v2 で起動)。
#   cd ~/Desktop/pai_final/host && bash run_percep.sh            # 各条件 10 回(N で変更)
#   SUITE=r3 bash run_percep.sh                                   # R3 回帰の確認(V5 各 20 回、V3 各 10 回)
# 事前: Docker Desktop と Ollama を起動。学習済みの認識器 ../docs/perception/model/recognizer.json が必要。
# 結果: ../docs/perception/runs/p1.csv(完了済みの試行は飛ばす)、経過ログ ../docs/perception/runs/logs/
set -u
cd "$(dirname "$0")"
DOCKER_DIR="$(pwd)/../docker"
source .venv/bin/activate
SUITE=${SUITE:-p1}
N=${N:-}
MODEL=../docs/perception/model/recognizer.json
[ -f "$MODEL" ] || { echo "$MODEL がありません(学習を先に行う)"; exit 1; }
cp "$MODEL" ../ros2_ws/src/ai_nav/config/recognizer.json
curl -s -m 3 localhost:11434 >/dev/null || { echo "Ollama が起動していません"; exit 1; }
SRC="source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash"
dexec() { (cd "$DOCKER_DIR" && docker compose -f compose.yaml exec -T -u ubuntu ros bash -c "$*" < /dev/null); }
sim_up() {
  dexec "$SRC && ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active && ros2 param get /plan_executor recognizer >/dev/null 2>&1" && nc -z localhost 9090
}
hard_restart() {
  echo "[$(date +%T)] シミュレータを(再)起動します(plan_executor_v2)"
  (cd "$DOCKER_DIR" && docker compose -f compose.multi.yaml stop >/dev/null 2>&1; docker compose -f compose.yaml up -d) || return 1
  dexec "bash /home/ubuntu/ros2_ws/stop_nav.sh" >/dev/null 2>&1
  dexec "$SRC && cd ~/ros2_ws && colcon build --symlink-install --packages-select ai_nav > /tmp/build.log 2>&1" || { echo "ビルド失敗"; return 1; }
  (cd "$DOCKER_DIR" && docker compose -f compose.yaml exec -T -d -u ubuntu ros setsid bash /home/ubuntu/ros2_ws/run_auto_nav.sh executor:=plan_executor_v2 < /dev/null)
  for i in $(seq 1 60); do
    sleep 5
    if sim_up; then echo "[$(date +%T)] 起動完了(約 $((i*5)) 秒)"; sleep 5; return 0; fi
  done
  echo "起動を確認できません(コンテナ内 /tmp/nav.log)"; return 1
}
caffeinate -i -w $$ &
hard_restart || exit 1
restarts=0
while true; do
  COMPOSE_FILE=compose.yaml python auto_percep.py --suite "$SUITE" ${N:+--n "$N"}; rc=$?
  case $rc in
    0) break;;
    75) restarts=$((restarts+1)); [ $restarts -gt 6 ] && { echo "再起動が続くため中止"; exit 1; }; hard_restart || exit 1;;
    *) echo "異常終了 (code $rc)"; exit $rc;;
  esac
done
echo "[$(date +%T)] 完了: ../docs/perception/runs/$SUITE.csv"
