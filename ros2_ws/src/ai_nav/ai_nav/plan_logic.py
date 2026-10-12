"""ROS に依存しない計画の検証ロジック(単体テスト可能にするため分離)。"""
import json
import math

import yaml

VALID_ACTIONS = ('goto', 'patrol', 'stop')
MAX_TARGETS = 8
MAX_LOOPS = 3


def load_places(path):
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    places = data['places']
    for pid, p in places.items():
        for k in ('x', 'y', 'yaw'):
            if not isinstance(p.get(k), (int, float)):
                raise ValueError(f'place {pid}: {k} must be a number')
    return places


def parse_plan(text, places):
    """JSON 文字列を検証し、(action, [place_id, ...]) を返す。不正なら ValueError。

    LLM の出力は信用しない前提で、ここで地点 id・件数・ループ数を制限する。
    """
    try:
        plan = json.loads(text)
    except json.JSONDecodeError as e:
        raise ValueError(f'invalid JSON: {e}') from e
    if not isinstance(plan, dict):
        raise ValueError('plan must be a JSON object')

    action = plan.get('action')
    if action not in VALID_ACTIONS:
        raise ValueError(f'unknown action: {action!r}')
    if action == 'stop':
        return 'stop', []

    targets = plan.get('targets')
    if not isinstance(targets, list) or not targets:
        raise ValueError('targets must be a non-empty list')
    if len(targets) > MAX_TARGETS:
        raise ValueError(f'too many targets (max {MAX_TARGETS})')
    unknown = [t for t in targets if t not in places]
    if unknown:
        raise ValueError(f'unknown place id(s): {unknown}')

    if action == 'patrol':
        loops = plan.get('loops', 1)
        if not isinstance(loops, int) or not 1 <= loops <= MAX_LOOPS:
            raise ValueError(f'loops must be an integer in 1..{MAX_LOOPS}')
        targets = targets * loops

    return action, list(targets)


def yaw_to_quaternion(yaw):
    """(z, w) を返す(平面上の回転のみ)。"""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def aisle_scan_hits(ranges, angle_min, angle_increment, range_min, range_max,
                    robot_xy, robot_yaw, aisle, half_length=1.2, half_width=0.3):
    """LiDAR の点のうち、通路の中心線上の帯(長さ 2*half_length、幅 2*half_width)に入る数。

    通路は x 方向に延びる。壁は通路中心から約 0.4 m 離れているので、half_width を 0.3 m に
    して壁を数えない。パレットは幅いっぱいを塞ぐため、手前の面が帯の中に点として現れる。
    地図とオドメトリは同じ原点(map->odom は恒等変換)として扱う。
    """
    hits = 0
    for i, r in enumerate(ranges):
        if not (range_min <= r <= range_max) or math.isinf(r) or math.isnan(r):
            continue
        a = robot_yaw + angle_min + i * angle_increment
        x = robot_xy[0] + r * math.cos(a)
        y = robot_xy[1] + r * math.sin(a)
        if abs(x - aisle['x']) <= half_length and abs(y - aisle['y']) <= half_width:
            hits += 1
    return hits
