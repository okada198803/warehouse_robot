"""停止位置から通路を特定する(位置による判定)。plan_executor_v2 が使う。ROS に依存しない。

元の plan_executor は「まだ経由地に着いていない通路」だけを候補にする。そのため、通路の経由地(中心)を
通過した後に通路の後半で止まると、通路を特定できず(unlocated)、LiDAR の確認も行われなかった
(複数台の失敗の型 C、6 件。docs/perception/R0_R6_ログ再解析.md)。
ここでは、経由地の到達とは無関係に、停止位置がどの通路の範囲に入っているかで判定する。
通過済みの通路も候補に含めるが、登録するかどうかは LiDAR の点数(しきい値)で決める(誤封鎖の防止は LiDAR が担う)。
"""
import math


def locate_aisle(pose, places, half_length=1.6, half_width=1.1):
    """pose (x, y) が入っている通路 id を返す。どの通路にも入っていなければ None。

    通路は x 方向に延び、中心 (x, y) から前後 half_length、左右 half_width の範囲とする。
    複数の範囲に入る場合は、中心線(y)に最も近い通路を選ぶ。
    half_width は南通路の外側(南壁から仕切りの脇まで)を含めるため 1.1 m とする。
    通路の間は棚で、ロボットは入れないため、範囲の重なりは棚の位置にしか生じない。
    """
    if pose is None:
        return None
    best, best_dy = None, None
    for pid, p in places.items():
        if p.get('kind') != 'aisle':
            continue
        dx = abs(pose[0] - p['x'])
        dy = abs(pose[1] - p['y'])
        if dx <= half_length and dy <= half_width and (best_dy is None or dy < best_dy):
            best, best_dy = pid, dy
    return best


def should_retry_abort(retries_done, max_retries):
    """Nav2 の中断(ABORTED)の後に、同じ目的地を再送するか。"""
    return retries_done < max_retries


def rear_clearance(ranges, angle_min, angle_increment, range_min, range_max, half_angle=0.7):
    """LiDAR から見て真後ろ ± half_angle [rad] の範囲で、最も近い点までの距離。点がなければ無限大。

    後退の前に後方を確かめるために使う(元の後退は後方を見ずに下がり、壁や隅に押し付けられて動けなくなった。
    docs/perception/M9_後退の後方確認.md)。LiDAR はロボット中心の 0.15 m 前にあるため、
    ロボットの後端(半径 約 0.2 m)は LiDAR から 約 0.35 m 後ろになる。
    """
    best = math.inf
    for i, r in enumerate(ranges):
        if r is None or not (range_min <= r <= range_max) or math.isinf(r) or math.isnan(r):
            continue
        a = angle_min + i * angle_increment
        d = math.atan2(math.sin(a - math.pi), math.cos(a - math.pi))   # 真後ろからのずれ
        if abs(d) <= half_angle and r < best:
            best = r
    return best
