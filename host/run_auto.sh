#!/bin/bash
# Gazebo 検証(V2 / V3 / V5 / V7 / V8 / V8b)を無人で実行する。シミュレータの起動・異常時の再起動も行う。
#   cd ~/Desktop/pai_final/host && ./run_auto.sh            # v2 v3 v5 v7 v8 を順に実行
#   ./run_auto.sh v3                                        # 一部だけ(v7 = LiDAR の感度の記録と解析)
#   N_V3=2 N_V5=2 ./run_auto.sh                             # 試運転(回数を減らす)
#   v8b = 荷物あり(cargo_sim 付きで起動し直す。v8 の後に実行する)
# 事前: Docker Desktop と Ollama を起動しておく。noVNC で手動起動した launch は閉じておく
#       (このスクリプトが stop_nav.sh で止めてから起動し直す)。
# 結果: ../docs/auto/*.csv(完了済みの試行は再実行で飛ばす)/ 経過ログ ../docs/auto/logs/
set -u
cd "$(dirname "$0")"
HOST_DIR=$(pwd)
DOCKER_DIR="$HOST_DIR/../docker"
source .venv/bin/activate
SUITES=("$@"); [ ${#SUITES[@]} -eq 0 ] && SUITES=(v2 v3 v5 v7 v8 v8b)
mkdir -p ../docs/auto/logs
LOG=../docs/auto/logs/run_auto.log

CARGO_SIM=0   # 1 のとき cargo:=true で起動する(v8b)

dexec() { (cd "$DOCKER_DIR" && docker compose exec -T -u ubuntu ros bash -c "$*"); }

hard_restart() {
  echo "[$(date +%T)] シミュレータを再起動します"
  (cd "$DOCKER_DIR" && docker compose up -d) || return 1
  dexec "bash /home/ubuntu/ros2_ws/stop_nav.sh" >/dev/null 2>&1
  (cd "$DOCKER_DIR" && docker compose exec -T -d -u ubuntu ros setsid bash /home/ubuntu/ros2_ws/run_auto_nav.sh $([ "$CARGO_SIM" = 1 ] && echo cargo:=true))
  for i in $(seq 1 60); do      # 最大 5 分待つ
    sleep 5
    if dexec "source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active && ros2 node list 2>/dev/null | grep -q plan_executor" \
       && nc -z localhost 9090; then
      echo "[$(date +%T)] 起動完了(約 $((i*5)) 秒)"; sleep 5; return 0
    fi
  done
  echo "[$(date +%T)] 起動を確認できません。コンテナ内 /tmp/nav.log を確認してください"; return 1
}

sim_up() {
  dexec "source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active && ros2 node list 2>/dev/null | grep -q plan_executor" \
    && nc -z localhost 9090
}

run_v7() {
  sim_up || hard_restart || return 1
  dexec "source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && python3 /home/ubuntu/ros2_ws/src/ai_nav/tools/record_scans.py" || return 1
  python analyze_v7.py
}

run_suite() {
  local s=$1 n_opt=""
  if [ "$s" != v8b ] && dexec "source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && ros2 node list 2>/dev/null | grep -q cargo_sim"; then
    CARGO_SIM=0; hard_restart || return 1      # 荷物の模擬が動いたままなら、荷物なしの構成で起動し直す
  fi
  if [ "$s" = v8b ]; then      # 荷物の模擬(cargo_sim)付きで起動し直す
    CARGO_SIM=1
    dexec "source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && ros2 node list 2>/dev/null | grep -q cargo_sim" || hard_restart || return 1
  fi
  [ "$s" = v7 ] && { run_v7; return $?; }
  case $s in v3) [ -n "${N_V3:-}" ] && n_opt="--n $N_V3";; v5) [ -n "${N_V5:-}" ] && n_opt="--n $N_V5";; v2) [ -n "${N_V2:-}" ] && n_opt="--n $N_V2";; v8) [ -n "${N_V8:-}" ] && n_opt="--n $N_V8";; v8b) [ -n "${N_V8B:-}" ] && n_opt="--n $N_V8B";; esac
  local restarts=0
  while true; do
    python auto_sim.py --suite "$s" $n_opt; rc=$?
    case $rc in
      0) return 0;;
      75) restarts=$((restarts+1))
          [ $restarts -gt 6 ] && { echo "再起動が続くため $s を中止"; return 1; }
          hard_restart || return 1;;
      *) echo "$s: 異常終了 (code $rc)"; return $rc;;
    esac
  done
}

{
  echo "=== 開始 $(date) / suites: ${SUITES[*]} ==="
  curl -s http://localhost:11434/api/tags >/dev/null || echo "警告: Ollama に接続できません(V2 以外は失敗します)"
  # sleep を防ぐ(このスクリプトの終了まで)
  caffeinate -i -w $$ &
  for s in "${SUITES[@]}"; do
    echo "--- $s ($(date +%T)) ---"
    run_suite "$s"
  done
  python summarize_auto.py
  [ -f ../docs/auto/v8.csv ] && python analyze_eta.py
  echo "=== 終了 $(date) ==="
} 2>&1 | tee -a "$LOG"
