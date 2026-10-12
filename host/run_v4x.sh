#!/bin/bash
# 解除の言い方の拡張(V4x、第 3 段)。必須化(--strict-schema)+ ロボット報告への返答では解除を受け付けない修正の確認。
# Ollama が必要。約 10〜15 分。temperature=0 のため各シナリオは 1 回ずつ。
#   cd ~/Desktop/pai_final/host && ./run_v4x.sh
set -u
cd "$(dirname "$0")"
source .venv/bin/activate
curl -s http://localhost:11434/api/tags >/dev/null || { echo "Ollama に接続できません"; exit 1; }
mkdir -p ../docs/auto/logs
{
  echo "## 1 基本シナリオ(必須化 + 報告への返答では解除しない)"; python measure_llm.py --suite base --reps 1 --strict-schema --tag base_strict2
  echo "## 2 V4 全体";                                          python measure_llm.py --suite v4 --reps 1 --strict-schema --tag V4_strict2
  echo "## 3 解除の言い方 規則 + LLM";                           python measure_llm.py --suite v4x --reps 1 --strict-schema --tag V4x_rules_strict2
  echo "## 4 解除の言い方 LLM のみ";                             python measure_llm.py --suite v4x --reps 1 --strict-schema --no-clear-rules --tag V4x_llm_strict2
  echo "## 5 経験の比較(V1)の確認";                              python measure_llm.py --suite v1 --reps 1 --strict-schema --tag V1_A_strict
} 2>&1 | tee ../docs/auto/logs/v4x_strict2.log
