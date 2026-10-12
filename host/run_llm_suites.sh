#!/bin/bash
# V1(経験の参照方法の比較)と V4(言い換え)を、ROS なしで連続実行する。Ollama が起動していること。
#   cd ~/Desktop/pai_final/host && ./run_llm_suites.sh [reps_v1=10] [reps_v4=5]
# 所要: おおよそ 1〜1.5 時間(モデルの速度による)。ログ: ../docs/auto/logs/llm_suites.log
set -u
cd "$(dirname "$0")"
source .venv/bin/activate
R1=${1:-10}; R4=${2:-5}
mkdir -p ../docs/auto/logs
LOG=../docs/auto/logs/llm_suites.log
curl -s http://localhost:11434/api/tags >/dev/null || { echo "Ollama に接続できません"; exit 1; }
cp -f experience.json experience.json.bak 2>/dev/null
{
  date
  echo "## 経験の作成(学習用シナリオ)"; python measure_llm.py --train || exit 1
  echo "## V1-A  経験なし";           python measure_llm.py --suite v1 --reps "$R1" --tag V1_A
  echo "## V1-B1 経験あり(完全一致のみ)"; python measure_llm.py --suite v1 --reps "$R1" --experience on --tag V1_B_exact
  echo "## V1-B2 経験あり(類似も参照)";   python measure_llm.py --suite v1 --reps "$R1" --experience similar --tag V1_B_similar
  echo "## V4 言い換え";               python measure_llm.py --suite v4 --reps "$R4" --tag V4
  date
} 2>&1 | tee "$LOG"
