#!/bin/bash
# 複数ロボット検証を 1 回の実行で行う。シミュレータの起動・異常時の再起動・集計まで自動。
#   cd ~/Desktop/pai_final/multi/host && ./run_multi_auto.sh              # C3 と C1 を交互に各 10 試行
#   ./run_multi_auto.sh C0,C3,C1                                          # 条件を指定(完了済みの試行は飛ばす)
#   N=3 ./run_multi_auto.sh C3                                            # 試運転(回数を減らす)
# 事前: Docker Desktop を起動。C1 を含むときは Ollama も起動(qwen2.5:7b)。単体構成のコンテナは止めておく。
# 結果: ../docs/multi/<条件>.csv と summary.md、ログ ../docs/multi/logs/
set -u
cd "$(dirname "$0")"
DOCKER_DIR="$(pwd)/../docker"
source ../../host/.venv/bin/activate
CONDS=${1:-C3,C1}
N=${N:-10}
mkdir -p ../docs/multi/logs
SRC="source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash"

dexec() { (cd "$DOCKER_DIR" && docker compose exec -T -u ubuntu ros bash -c "$*"); }

sim_up() {
  dexec "$SRC && ros2 lifecycle get /r1/bt_navigator 2>/dev/null | grep -q active && ros2 lifecycle get /r2/bt_navigator 2>/dev/null | grep -q active && [ \$(ros2 node list 2>/dev/null | grep -c plan_executor) = 2 ]" \
    && nc -z localhost 9091
}

hard_restart() {
  echo "[$(date +%T)] シミュレータを(再)起動します"
  (cd "$DOCKER_DIR" && docker compose up -d) || return 1
  dexec "bash /home/ubuntu/ros2_ws/stop_nav.sh" >/dev/null 2>&1      # 停止と起動は別のコマンドにする(pkill -f が自分を止めるため)
  (cd "$DOCKER_DIR" && docker compose exec -T -d -u ubuntu ros setsid bash /home/ubuntu/ros2_ws/run_multi.sh)
  for i in $(seq 1 60); do      # 最大 5 分待つ
    sleep 5
    if sim_up; then echo "[$(date +%T)] 起動完了(約 $((i*5)) 秒)"; sleep 5; return 0; fi
  done
  echo "[$(date +%T)] 起動を確認できません。コンテナ内 /tmp/nav.log を確認してください"; return 1
}

case "$CONDS" in *C1*) curl -s -m 3 localhost:11434 >/dev/null || { echo "Ollama が起動していません(C1 に必要)。起動してから再実行してください"; exit 1; };; esac

restarts=0
while true; do
  sim_up || hard_restart || exit 1
  python auto_multi.py --conds "$CONDS" --n "$N"; rc=$?
  case $rc in
    0) break;;
    75) restarts=$((restarts+1))
        [ $restarts -gt 6 ] && { echo "再起動が続くため中止"; exit 1; }
        hard_restart || exit 1;;
    *) echo "異常終了 (code $rc)"; exit $rc;;
  esac
done
python analyze_multi.py
