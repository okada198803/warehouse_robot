"""到着できた計画の記録(経験)。次の判断のプロンプトに例として入れる。

モデルの重みは更新しない(LLM は固定)。「同じ通行止めの状況で、過去に最短で到着できた計画」を
参考として LLM に見せることで、初回案が検証を通る割合を上げる(文脈内の経験再利用)。

保存条件: 検証を通り、最短(許容差内)で、実際に到着(status: done)した計画だけ。
検索: 通行止めの通路の組が一致する経験を優先し、新しいものから最大 k 件。
"""
import json
import os


class Experience:

    def __init__(self, path=None, k=2, read_only=False, similar=False):
        self.similar = similar
        self.path = path
        self.k = k
        self.read_only = read_only
        self.items = []
        if path and os.path.exists(path):
            with open(path, encoding='utf-8') as f:
                self.items = json.load(f)

    def add(self, blocked, instruction, targets, length):
        if self.read_only:
            return False
        item = {'blocked': sorted(blocked), 'instruction': instruction,
                'targets': list(targets), 'length': round(length, 1)}
        # 同じ通行止め・同じ計画は重複させない(最新の指示文で置き換える)
        self.items = [i for i in self.items
                      if not (i['blocked'] == item['blocked'] and i['targets'] == item['targets'])]
        self.items.append(item)
        self.save()
        return True

    def save(self):
        if not self.path:
            return
        with open(self.path, 'w', encoding='utf-8') as f:
            json.dump(self.items, f, ensure_ascii=False, indent=1)

    def retrieve(self, blocked, similar=False):
        """通行止めの組が一致する経験を、新しい順に最大 k 件。

        similar=True: 一致が無いとき、通行止めの重なり(Jaccard)が大きい経験を代わりに返す
        (返す項目に 'similar': True を付ける)。重なりが 0 のものは返さない。
        """
        key = sorted(blocked)
        hits = [i for i in self.items if i['blocked'] == key]
        if hits or not similar:
            return list(reversed(hits))[:self.k]
        ks = set(key)

        def score(i):
            b = set(i['blocked'])
            u = ks | b
            return len(ks & b) / len(u) if u else 0.0
        ranked = sorted(reversed(self.items), key=score, reverse=True)   # 同点は新しい順
        return [{**i, 'similar': True} for i in ranked if score(i) > 0][:self.k]

    def __len__(self):
        return len(self.items)
