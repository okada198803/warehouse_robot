#!/bin/bash
# 複数ロボット検証を 1 回の実行で行う。シミュレータの起動・異常時の再起動・集計まで自動。
#   cd ~/Desktop/pai_final/multi/host && ./run_multi_auto.sh              # C3 と C1 を交互に各 10 試行
#   ./run_multi_auto.sh C2w,C2                                            # 広い通路のワールド(warehouse_wide)で実行
#   ./run_multi_auto.sh C3,C1,C2w,C2                                      # 全条件(ワールドごとに段階を分け、切り替え時に再起動)
#   N=3 ./run_multi_auto.sh C3                                            # 試運転(回数を減らす)
#   SCEN=same,cross,wait ./run_multi_auto.sh C0,C1,C3,C2                  # シナリオを指定(既定 opp = 正面すれ違い。同方向 same / 交差 cross / 待機 wait)
#   cross では C2(南通路の車線規則)は経路が南通路を通らないため対象外(自動で飛ばす)
# 事前: Docker Desktop を起動。C1 を含むときは Ollama も起動(qwen2.5:7b)。単体構成のコンテナは止めておく。
# 結果: ../docs/multi/<条件>.csv(opp)・<条件>_<シナリオ>.csv と summary*.md、ログ ../docs/multi/logs/(完了済みの試行は飛ばす)
set -u
cd "$(dirname "$0")"
DOCKER_DIR="$(pwd)/../docker"
export COMPOSE_FILE=compose.multi.yaml     # multi 用コンテナ(pai-ros-multi)。docker compose と ros_exec.sh がこれを使う
if [ -f .venv/bin/activate ]; then source .venv/bin/activate; else source ../../host/.venv/bin/activate; fi   # main では host/.venv、ワークツリーでは main 側のもの
CONDS=${1:-C3,C1}
N=${N:-10}
SCEN=${SCEN:-opp}
mkdir -p ../docs/multi/logs
SRC="source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash"

dexec() { (cd "$DOCKER_DIR" && docker compose exec -T -u ubuntu ros bash -c "$*"); }

sim_up() {   # $1 = 期待するワールド
  dexec "$SRC && ros2 lifecycle get /r1/bt_navigator 2>/dev/null | grep -q active && ros2 lifecycle get /r2/bt_navigator 2>/dev/null | grep -q active && [ \$(ros2 node list 2>/dev/null | grep -c plan_executor) = 2 ] && [ \"\$(cat /tmp/multi_world 2>/dev/null)\" = $1 ]" \
    && nc -z localhost 9091
}

hard_restart() {   # $1 = ワールド
  echo "[$(date +%T)] シミュレータを(再)起動します(world=$1)"
  (cd "$DOCKER_DIR" && docker compose up -d) || return 1
  dexec "bash /home/ubuntu/ros2_ws/stop_nav.sh" >/dev/null 2>&1      # 停止と起動は別のコマンドにする(pkill -f が自分を止めるため)
  dexec "$SRC && cd ~/ros2_ws && colcon build --symlink-install > /tmp/build.log 2>&1" || { echo "colcon build に失敗(コンテナ内 /tmp/build.log)"; return 1; }
  dexec "echo $1 > /tmp/multi_world"
  (cd "$DOCKER_DIR" && docker compose exec -T -d -u ubuntu ros setsid bash /home/ubuntu/ros2_ws/run_multi.sh world:=$1)
  for i in $(seq 1 60); do      # 最大 5 分待つ
    sleep 5
    if sim_up "$1"; then echo "[$(date +%T)] 起動完了(約 $((i*5)) 秒)"; sleep 5; return 0; fi
  done
  echo "[$(date +%T)] 起動を確認できません。コンテナ内 /tmp/nav.log を確認してください"; return 1
}

run_phase() {   # $1 = 条件(カンマ区切り) $2 = ワールド $3 = シナリオ
  [ -z "$1" ] && return 0
  echo "=== 段階: シナリオ $3 / 条件 $1 / world=$2 ==="
  local restarts=0
  while true; do
    sim_up "$2" || hard_restart "$2" || return 1
    python auto_multi.py --conds "$1" --n "$N" --scenario "$3"; rc=$?
    case $rc in
      0) return 0;;
      75) restarts=$((restarts+1))
          [ $restarts -gt 6 ] && { echo "再起動が続くため中止"; return 1; }
          hard_restart "$2" || return 1;;
      *) echo "異常終了 (code $rc)"; return $rc;;
    esac
  done
}

case "$CONDS" in *C1*) curl -s -m 3 localhost:11434 >/dev/null || { echo "Ollama が起動していません(C1 に必要)。起動してから再実行してください"; exit 1; };; esac

BASE=$(echo "$CONDS" | tr ',' '\n' | grep -v '^C2' | paste -sd, -)
WIDE=$(echo "$CONDS" | tr ',' '\n' | grep '^C2' | paste -sd, -)
for S in $(echo "$SCEN" | tr ',' ' '); do      # ワールドごとに段階を分け(再起動は 1 回)、その中でシナリオを順に実行する
  run_phase "$BASE" warehouse "$S" || exit 1
done
for S in $(echo "$SCEN" | tr ',' ' '); do
  [ "$S" = cross ] && continue                  # cross では C2 は対象外
  run_phase "$WIDE" warehouse_wide "$S" || exit 1
done
python analyze_multi.py
