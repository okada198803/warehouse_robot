#!/bin/bash
# 複数ロボット検証を 1 回の実行で行う。シミュレータの起動・異常時の再起動・集計まで自動。
#   cd ~/Desktop/pai_final/multi/host && ./run_multi_auto.sh              # C3 と C1 を交互に各 10 試行
#   ./run_multi_auto.sh C2w,C2                                            # 広い通路のワールド(warehouse_wide)で実行
#   ./run_multi_auto.sh C3,C1,C2w,C2                                      # 全条件(ワールドごとに段階を分け、切り替え時に再起動)
#   N=3 ./run_multi_auto.sh C3                                            # 試運転(回数を減らす)
#   EXECUTOR=plan_executor_v2 MULTI_SUFFIX=_fix SCEN=dead,waitj ./run_multi_auto.sh C0,C3   # 型 B・C の規則の修正版で測る(別ファイルに記録)
#   EXECUTOR=plan_executor_v2 RECOGNIZER=learned MULTI_SUFFIX=_fix_r3 N=3 ./run_multi_auto.sh C0,C1,C3,C2   # R3: 学習した認識器で(停止原因の判定だけが変わる。_fix を含む名前は元の集計から除く)
#   SCEN=same,cross,wait ./run_multi_auto.sh C0,C1,C3,C2                  # シナリオを指定(既定 opp = 正面すれ違い。同方向 same / 交差 cross / 待機 wait)
#   cross・crossj・dest では C2(南通路の車線規則)は対象外(自動で飛ばす)。追加: stag(時間差)・dest(同じ目的地)・dead(恒久停止)・oppj samej crossj waitj(ばらつき)
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
export EXECUTOR=${EXECUTOR:-plan_executor}   # plan_executor_v2: 型 B・C の規則の修正(認識層の検証)。元のデータと混ぜないため MULTI_SUFFIX が必須
if [ "$EXECUTOR" != plan_executor ] && [ -z "${MULTI_SUFFIX:-}" ]; then
  echo "EXECUTOR=$EXECUTOR では MULTI_SUFFIX(例 _fix)を指定してください(元の測定データを上書きしないため)"; exit 1
fi
export RECOGNIZER=${RECOGNIZER:-}            # plan_executor_v2 の recognizer(rule / rule_offset / learned)。空なら既定(rule)
if [ -n "$RECOGNIZER" ]; then
  [ "$EXECUTOR" = plan_executor_v2 ] || { echo "RECOGNIZER は EXECUTOR=plan_executor_v2 のときだけ指定できます"; exit 1; }
  if [ "$RECOGNIZER" = learned ]; then
    cp ../docs/perception/model/recognizer.json ../ros2_ws/src/ai_nav/config/recognizer.json || exit 1
  fi
fi
mkdir -p ../docs/multi/logs
SRC="source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash"

# docker exec が応答しなくなることがあるため、一定時間(既定 240 秒、TMO で変更)で打ち切る(macOS に timeout は無いので perl の alarm を使う)
# 時間切れで子プロセスを TERM → KILL する(docker compose は SIGALRM で止まらないことがあるため、alarm を直接は使わない)
tmo() { perl -e 'my $t=shift; my $pid=fork; if(!$pid){exec @ARGV; exit 127} $SIG{ALRM}=sub{kill "TERM",$pid; sleep 3; kill "KILL",$pid; waitpid($pid,0); exit 124}; alarm $t; waitpid($pid,0); exit($?>>8)' "$@"; }
dexec() { (cd "$DOCKER_DIR" && tmo "${TMO:-240}" docker compose exec -T -u ubuntu ros bash -c "$*"); }

sim_up() {   # $1 = 期待するワールド
  TMO=90 dexec "$SRC && ros2 lifecycle get /r1/bt_navigator 2>/dev/null | grep -q active && ros2 lifecycle get /r2/bt_navigator 2>/dev/null | grep -q active && [ \$(ros2 node list 2>/dev/null | grep -c plan_executor) = 2 ] && [ \"\$(cat /tmp/multi_world 2>/dev/null)\" = \"$1 $EXECUTOR $RECOGNIZER\" ]" \
    && nc -z localhost 9091
}

hard_restart() {   # $1 = ワールド
  echo "[$(date +%T)] シミュレータを(再)起動します(world=$1)"
  (cd "$DOCKER_DIR" && tmo 180 docker compose up -d && tmo 180 docker compose restart) || return 1     # restart で、固まったコンテナ内の処理もすべて止める
  dexec "bash /home/ubuntu/ros2_ws/stop_nav.sh" >/dev/null 2>&1      # 停止と起動は別のコマンドにする(pkill -f が自分を止めるため)
  dexec "$SRC && cd ~/ros2_ws && colcon build --symlink-install > /tmp/build.log 2>&1" || { echo "colcon build に失敗(コンテナ内 /tmp/build.log)"; return 1; }
  dexec "echo $1 $EXECUTOR $RECOGNIZER > /tmp/multi_world"
  (cd "$DOCKER_DIR" && tmo 90 docker compose exec -T -d -u ubuntu ros setsid bash /home/ubuntu/ros2_ws/run_multi.sh world:=$1 executor:=$EXECUTOR)
  for i in $(seq 1 60); do      # 最大 5 分待つ
    sleep 5
    if sim_up "$1"; then
      echo "[$(date +%T)] 起動完了(約 $((i*5)) 秒)"; sleep 5
      if [ -n "$RECOGNIZER" ]; then      # 起動引数では渡せないため、起動後に各台のパラメータを設定する
        for ns in $(TMO=60 dexec "$SRC && ros2 node list 2>/dev/null | grep plan_executor" | tr -d '\r'); do
          TMO=60 dexec "$SRC && ros2 param set $ns recognizer $RECOGNIZER" | grep -q successful || { echo "$ns の recognizer を設定できません"; return 1; }
        done
        echo "[$(date +%T)] recognizer=$RECOGNIZER を設定"
      fi
      return 0
    fi
  done
  echo "[$(date +%T)] 起動を確認できません。コンテナ内 /tmp/nav.log を確認してください"; return 1
}

run_phase() {   # $1 = 条件(カンマ区切り) $2 = ワールド $3 = シナリオ
  [ -z "$1" ] && return 0
  echo "=== 段階: シナリオ $3 / 条件 $1 / world=$2 ==="
  local restarts=0
  hard_restart "$2" || return 1      # 段階の始めに必ず再起動する(長時間の実行でコンテナ内に溜まった処理を消す)
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

case "$CONDS" in *C1*|*C4*) curl -s -m 3 localhost:11434 >/dev/null || { echo "Ollama が起動していません(C1 に必要)。起動してから再実行してください"; exit 1; };; esac

BASE=$(echo "$CONDS" | tr ',' '\n' | grep -v '^C2' | paste -sd, -)
WIDE=$(echo "$CONDS" | tr ',' '\n' | grep '^C2' | paste -sd, -)
for S in $(echo "$SCEN" | tr ',' ' '); do      # ワールドごとに段階を分け(再起動は 1 回)、その中でシナリオを順に実行する
  python - "$S" "$BASE" <<'PY' && continue      # 残りの試行が無い段階は、再起動せずに飛ばす
import os, sys, csv
s, conds = sys.argv[1], [c for c in sys.argv[2].split(',') if c]
n = int(os.environ.get('N', '10'))
def done(c):
    p = os.path.join('..', 'docs', 'multi', f"{c}{'' if s == 'opp' else '_' + s}{os.environ.get('MULTI_SUFFIX', '')}.csv")
    return len({r['trial'] for r in csv.DictReader(open(p, encoding='utf-8-sig'))}) if os.path.exists(p) else 0
sys.exit(0 if conds and all(done(c) >= n for c in conds) else 1)
PY
  run_phase "$BASE" warehouse "$S" || exit 1
done
for S in $(echo "$SCEN" | tr ',' ' '); do
  case "$S" in cross|crossj|dest) continue;; esac     # 経路が南通路を通らない(または規則が無関係な)シナリオでは C2 は対象外
  run_phase "$WIDE" warehouse_wide "$S" || exit 1
done
python analyze_multi.py
