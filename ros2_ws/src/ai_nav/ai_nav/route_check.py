"""地図上の経路検証(numpy のみ。ROS 非依存)。

Nav2 の大域計画は静的地図だけを見て最短経路を引くため、通行止めの通路を経由地なしで
指定すると同じ通路に突っ込む。ここでは各区間の「静的地図上の最短経路」を求め、
通行止め通路を通るかどうかを判定する。LLM が出した計画の検証と、迂回案の算出に使う。
"""
import heapq
import math
import os

import numpy as np
import yaml


def load_map(yaml_path):
    with open(yaml_path, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    pgm = os.path.join(os.path.dirname(yaml_path), meta['image'])
    with open(pgm, 'rb') as f:
        data = f.read()
    # P5 ヘッダ(コメント行なしを想定)
    parts = data.split(maxsplit=4)
    if parts[0] != b'P5':
        raise ValueError('only binary PGM (P5) is supported')
    w, h = int(parts[1]), int(parts[2])
    img = np.frombuffer(parts[4][:w * h], dtype=np.uint8).reshape(h, w)
    occ_th = meta.get('occupied_thresh', 0.65)
    occupancy = (255 - img.astype(float)) / 255.0
    occupied = occupancy > occ_th
    unknown = (occupancy > meta.get('free_thresh', 0.25)) & ~occupied
    return occupied | unknown, meta['origin'][:2], meta['resolution']


def _dilate(mask, radius_px):
    """円形カーネルで膨張(scipy を使わない簡易実装)。"""
    out = mask.copy()
    r = int(math.ceil(radius_px))
    h, w = mask.shape
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > radius_px * radius_px:
                continue
            src = mask[max(0, -dy):h - max(0, dy), max(0, -dx):w - max(0, dx)]
            out[max(0, dy):h - max(0, -dy), max(0, dx):w - max(0, -dx)] |= src
    return out


class RouteChecker:

    def __init__(self, map_yaml, places, robot_radius=0.20, inflation=0.35,
                 block_radius=0.55, hit_radius=0.30):
        blocked, origin, res = load_map(map_yaml)
        self.res = res
        self.ox, self.oy = origin
        self.h, self.w = blocked.shape
        self.places = places
        self.block_radius = block_radius   # 迂回計算で通行不可にする半径
        self.hit_radius = hit_radius       # 「通路を通過した」と判定する半径(通路中心の付近)
        self.lethal = _dilate(blocked, robot_radius / res)
        near_wall = _dilate(blocked, inflation / res) & ~self.lethal
        # 壁際を通りにくくする(Nav2 の inflation に近い振る舞い)
        self.cost = np.where(near_wall, 2.0, 1.0)

    # ---------- 座標変換 ----------
    def to_cell(self, x, y):
        c = int((x - self.ox) / self.res)
        r = self.h - 1 - int((y - self.oy) / self.res)
        return r, c

    def to_xy(self, r, c):
        return (self.ox + (c + 0.5) * self.res, self.oy + (self.h - 1 - r + 0.5) * self.res)

    def _snap(self, cell):
        """障害物上の点を最寄りの通行可能セルへ寄せる。"""
        r0, c0 = cell
        if 0 <= r0 < self.h and 0 <= c0 < self.w and not self.lethal[r0, c0]:
            return cell
        for rad in range(1, 20):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    r, c = r0 + dr, c0 + dc
                    if 0 <= r < self.h and 0 <= c < self.w and not self.lethal[r, c]:
                        return r, c
        raise ValueError(f'no free cell near {cell}')

    def xy_of(self, target):
        """地点 id または (x, y) を座標にする。"""
        if isinstance(target, str):
            p = self.places[target]
            return float(p['x']), float(p['y'])
        return float(target[0]), float(target[1])

    # ---------- 経路探索 ----------
    def shortest_path(self, start, goal, avoid=()):
        """静的地図上の最短経路(セル列)と長さ[m]。avoid の通路は通行不可として扱う。"""
        lethal = self.lethal
        if avoid:
            lethal = lethal | self._discs(avoid)
        s = self._snap(self.to_cell(*self.xy_of(start)))
        g = self._snap(self.to_cell(*self.xy_of(goal)))
        if lethal[s] or lethal[g]:
            return None, math.inf
        dist = {s: 0.0}
        prev = {}
        pq = [(0.0, s)]
        steps = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
                 (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
        while pq:
            d, cur = heapq.heappop(pq)
            if cur == g:
                break
            if d > dist[cur]:
                continue
            r, c = cur
            for dr, dc, k in steps:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < self.h and 0 <= nc < self.w) or lethal[nr, nc]:
                    continue
                nd = d + k * self.cost[nr, nc]
                if nd < dist.get((nr, nc), math.inf):
                    dist[(nr, nc)] = nd
                    prev[(nr, nc)] = cur
                    heapq.heappush(pq, (nd, (nr, nc)))
        if g not in dist:
            return None, math.inf
        path = [g]
        while path[-1] != s:
            path.append(prev[path[-1]])
        path.reverse()
        length = sum(math.dist(a, b) for a, b in zip(path, path[1:])) * self.res
        return path, length

    def _discs(self, aisle_ids):
        yy, xx = np.mgrid[0:self.h, 0:self.w]
        cx = self.ox + (xx + 0.5) * self.res
        cy = self.oy + (self.h - 1 - yy + 0.5) * self.res
        mask = np.zeros((self.h, self.w), dtype=bool)
        for a in aisle_ids:
            x, y = self.xy_of(a)
            mask |= (cx - x) ** 2 + (cy - y) ** 2 < self.block_radius ** 2
        return mask

    def aisles_on(self, path):
        """経路が通過する aisle 地点 id の集合。"""
        hit = set()
        for pid, p in self.places.items():
            if p.get('kind') != 'aisle':
                continue
            for r, c in path:
                x, y = self.to_xy(r, c)
                if (x - p['x']) ** 2 + (y - p['y']) ** 2 < self.hit_radius ** 2:
                    hit.add(pid)
                    break
        return hit

    def aisles_in_order(self, path):
        """経路が通過する aisle 地点 id を、通過順に並べたリスト。"""
        order = []
        for r, c in path:
            x, y = self.to_xy(r, c)
            for pid, p in self.places.items():
                if p.get('kind') != 'aisle' or pid in order:
                    continue
                if (x - p['x']) ** 2 + (y - p['y']) ** 2 < self.hit_radius ** 2:
                    order.append(pid)
        return order

    # ---------- 計画の検証 ----------
    def check_plan(self, start, targets, blocked):
        """問題点のリスト(日本語)と総距離[m]を返す。問題がなければリストは空。"""
        problems = []
        total = 0.0
        prev = start
        for t in targets:
            if t in blocked:
                problems.append(f'{self.places[t]["name"]}は通行止めのため目的地にできない')
                prev = t
                continue
            path, length = self.shortest_path(prev, t)
            if path is None:
                problems.append(f'{self._label(prev)} → {self._label(t)} の経路が地図上に存在しない')
            else:
                total += length
                crossed = self.aisles_on(path) & set(blocked)
                for a in sorted(crossed):
                    problems.append(
                        f'{self._label(prev)} → {self._label(t)} の最短経路が通行止めの'
                        f'{self.places[a]["name"]}を通る。別の通路を経由地に入れること')
            prev = t
        return problems, total

    def best_plan(self, start, goal, blocked):
        """通行止めを避ける最短の計画(地点 id のリスト)と総距離[m]。不可能なら (None, inf)。"""
        candidates = [[goal]] + [[a, goal] for a, p in self.places.items()
                                 if p.get('kind') == 'aisle' and a not in blocked]
        best, best_len = None, math.inf
        for plan in candidates:
            problems, length = self.check_plan(start, plan, blocked)
            if not problems and length < best_len:
                best, best_len = plan, length
        return best, best_len

    def suggest_detour(self, start, goal, blocked):
        """通行止めを避ける最短の計画(地点 id のリスト)。不可能なら None。"""
        return self.best_plan(start, goal, blocked)[0]

    def _label(self, t):
        return self.places[t]['name'] if isinstance(t, str) else '現在地'
