"""解除の言い方に対する LLM の生の出力を確認する(Ollama が必要)。
    python debug_clear.py [--hint] [--no-rules] "南通路は復旧した" ...
"""
import sys
from llm_commander import make_llm
from planner import Planner

args = sys.argv[1:]
hint = '--hint' in args
rules = '--no-rules' not in args
loose = '--loose' in args
texts = [a for a in args if not a.startswith('--')] or ['南通路はもう大丈夫になった', '南通路は復旧した', '南通路を塞いでいたものはなくなりました']
for t in texts:
    p = Planner('warehouse', make_llm('qwen2.5:7b'), clear_rules=rules, clear_hint=hint, strict_schema=not loose)
    p.blocked = {'south_aisle'}
    d = p.handle_user(t)
    print(f'\n[{t}] source={d.source} blocked={sorted(p.blocked)} cleared={sorted(p.cleared)}')
    for line in d.log:
        print('   ', line)
    print('    reply:', d.reply)
