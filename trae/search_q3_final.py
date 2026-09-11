# -*- coding: utf-8 -*-
"""
B 题问题3 终版搜索算法 v2(自适应频道跳过 + 早终止 + 2-opt路径)

优化点:
1. 自适应频道跳过: 某频道已有 >=2 次 direction 且最大交会角 >=60°,
   后续站不再扫该频道(省动作)
2. 无信号早终止: 某频道连续 5 站 no_signal, 后续站跳过(省动作)
3. 清除路径 2-opt: 对最近邻路径做 2-opt 局部优化(省虚拟时间)
4. 中位数交会定位: 对所有有效观测对求交会点取中位数(抗离群点)
5. 清除失败精扫: 在名义交会点附近 30m/50m 螺旋网格精扫
"""
import argparse
import json
import math
import os
import sys

from robot_client import RobotClient

# === 全局常量 ===
TARGET_RADIUS = 1800.0
MOVE_SPEED = 5.0  # m/s
REAL_TIME_RESERVE = 15.0  # 现实时间安全余量(秒)
MIN_CUT_ANGLE = 30.0  # 交会角小于此值则跳过清除
MAX_CLEAR_DIST_FROM_ORIGIN = 1800.0  # 不清除圆域外的名义交点

# 自适应跳过阈值
TRIANGULATE_MIN_ANGLE = 60.0  # 交会角达此值且 >=2 次观测 -> 后续站跳过
# 注意: 不使用无信号早终止! 前几站无信号不代表后续站也无信号,
# 因为源可能在后半段站的覆盖方向. 必须扫描完全部9站才能判定无源.

# 共享九站: 中心 + 半径 950 米圆周 8 个等角点
RING_RADIUS = 950.0
PROBE_POINTS = [(0.0, 0.0)] + [
    (RING_RADIUS * math.cos(i * math.pi / 4.0),
     RING_RADIUS * math.sin(i * math.pi / 4.0)) for i in range(8)
]

# 问题二闭式解: Q_± = S + 843.0348 u ± 545.5284 v
Q_FORWARD = 843.0348
Q_LATERAL = 545.5284


def norm_angle(deg):
    return deg % 360.0


def angle_diff(a, b):
    """最小夹角(0-180)"""
    d = abs(a - b) % 360
    return 360 - d if d > 180 else d


def ray_direction(deg):
    r = math.radians(deg)
    return math.cos(r), math.sin(r)


def move_along_bearing(x, y, bearing_deg, dist_m):
    rad = math.radians(bearing_deg)
    return x + dist_m * math.cos(rad), y + dist_m * math.sin(rad)


def pick_q_pm_point(origin, bearing_deg, current_pos):
    """从 S 出发, 沿 bearing 方向前进 Q_FORWARD, 再向 ± 方向偏移 Q_LATERAL.
    返回离 current_pos 较近者(避免绕路)."""
    p_plus = move_along_bearing(origin[0], origin[1], bearing_deg, Q_FORWARD)
    p_plus = move_along_bearing(p_plus[0], p_plus[1],
                                (bearing_deg + 90) % 360, Q_LATERAL)
    p_minus = move_along_bearing(origin[0], origin[1], bearing_deg, Q_FORWARD)
    p_minus = move_along_bearing(p_minus[0], p_minus[1],
                                 (bearing_deg - 90) % 360, Q_LATERAL)
    d_p = math.hypot(p_plus[0] - current_pos[0], p_plus[1] - current_pos[1])
    d_m = math.hypot(p_minus[0] - current_pos[0], p_minus[1] - current_pos[1])
    return p_plus if d_p <= d_m else p_minus


def triangulate(p1, theta1_deg, p2, theta2_deg):
    """两条射线交会点; 无交会或反向时返回 None."""
    x1, y1 = p1
    x2, y2 = p2
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    d1x, d1y = math.cos(t1), math.sin(t1)
    d2x, d2y = math.cos(t2), math.sin(t2)
    D = -d1x * d2y + d2x * d1y
    if abs(D) < 1e-9:
        return None
    dx = x2 - x1
    dy = y2 - y1
    s = (-dx * d2y + d2x * dy) / D
    if s < -1.0:
        return None
    return (x1 + s * d1x, y1 + s * d1y)


def best_intersection(observations):
    """从观测集合 [(point, bearing), ...] 中选交会角最大的一对."""
    best = None
    best_angle = 0.0
    for i in range(len(observations)):
        for j in range(i + 1, len(observations)):
            p1, a1 = observations[i]
            p2, a2 = observations[j]
            inter = triangulate(p1, a1, p2, a2)
            if inter is None:
                continue
            ang = angle_diff(a1, a2)
            if ang >= MIN_CUT_ANGLE and ang > best_angle:
                best = inter
                best_angle = ang
    return best, best_angle


def median_intersection(observations):
    """稳健定位: 对所有有效观测对求交会点, 取中位数(抗离群点)."""
    pts = []
    angs = []
    for i in range(len(observations)):
        for j in range(i + 1, len(observations)):
            p1, a1 = observations[i]
            p2, a2 = observations[j]
            inter = triangulate(p1, a1, p2, a2)
            if inter is None:
                continue
            ang = angle_diff(a1, a2)
            if ang < MIN_CUT_ANGLE:
                continue
            pts.append(inter)
            angs.append(ang)
    if not pts:
        return None, 0.0
    xs = sorted(p[0] for p in pts)
    ys = sorted(p[1] for p in pts)
    n = len(pts)
    if n % 2 == 1:
        mx = xs[n // 2]
        my = ys[n // 2]
    else:
        mx = (xs[n // 2 - 1] + xs[n // 2]) / 2
        my = (ys[n // 2 - 1] + ys[n // 2]) / 2
    return (mx, my), max(angs) if angs else 0.0


def max_intersection_angle(observations):
    """返回当前观测集合中最大交会角(不计算交会点)."""
    best = 0.0
    for i in range(len(observations)):
        for j in range(i + 1, len(observations)):
            _, a1 = observations[i]
            _, a2 = observations[j]
            ang = angle_diff(a1, a2)
            if ang > best:
                best = ang
    return best


def refine_clear(client, nominal_pt, channel, stats, search_radius=30.0,
                 step=10.0, max_measurements=8):
    """清除失败后的有限精扫: 在名义交会点附近做网格 measure.

    改进:
    1. 最多 max_measurements 次测量(默认8), 避免无限精扫浪费虚拟时间
    2. 若精扫中获得新的 direction 观测, 用它+原观测重新定位
    """
    cx, cy = nominal_pt
    rings = int(search_radius / step)
    new_obs = []  # 精扫中获得的新 direction 观测
    count = 0
    # 先试中心点
    mr, svd = measure_at(client, (cx, cy), channel, stats)
    count += 1
    if mr == "near":
        return try_clear_at(client, (cx, cy), channel, stats)
    if mr == "direction":
        new_obs.append(((cx, cy), svd))
    # 螺旋向外
    for ring in range(1, rings + 1):
        if count >= max_measurements:
            break
        r = ring * step
        for k in range(8):
            if count >= max_measurements:
                break
            ang = k * math.pi / 4
            px = cx + r * math.cos(ang)
            py = cy + r * math.sin(ang)
            if math.hypot(px, py) > TARGET_RADIUS:
                continue
            mr, svd = measure_at(client, (px, py), channel, stats)
            count += 1
            if mr == "near":
                return try_clear_at(client, (px, py), channel, stats)
            if mr == "direction":
                new_obs.append(((px, py), svd))
    # 若有 >=2 个新 direction 观测, 重新定位并尝试清除
    if len(new_obs) >= 2:
        inter, ang = median_intersection(new_obs)
        if inter is not None and \
           math.hypot(inter[0], inter[1]) <= MAX_CLEAR_DIST_FROM_ORIGIN:
            return try_clear_at(client, inter, channel, stats)
    return False


def nearest_neighbor_order(start, targets):
    """targets 是 (channel, x, y) 三元组列表, 返回贪心最近邻顺序."""
    remaining = list(targets)
    ordered = []
    current = start
    while remaining:
        item = min(remaining,
                    key=lambda v: math.hypot(v[1] - current[0], v[2] - current[1]))
        remaining.remove(item)
        ordered.append(item)
        current = (item[1], item[2])
    return ordered


def two_opt(route, start):
    """对最近邻路径做 2-opt 局部优化.

    route: [(channel, x, y), ...] 已排好序的目标列表
    start: (x, y) 起始位置
    返回优化后的路径.
    """
    if len(route) < 4:
        return route

    def route_distance(r):
        d = 0
        cur = start
        for _, x, y in r:
            d += math.hypot(x - cur[0], y - cur[1])
            cur = (x, y)
        return d

    best = list(route)
    best_dist = route_distance(best)
    improved = True
    iterations = 0
    max_iter = 100
    while improved and iterations < max_iter:
        improved = False
        iterations += 1
        for i in range(len(best) - 1):
            for j in range(i + 1, len(best)):
                if j - i == 1:
                    continue  # 相邻, 2-opt 无意义
                new_route = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                new_dist = route_distance(new_route)
                if new_dist < best_dist - 1e-6:
                    best = new_route
                    best_dist = new_dist
                    improved = True
    return best


def measure_at(client, point, channel, stats):
    """在 point 处检测 channel, 返回 (measure_result, svd_deg 或 None)."""
    resp = client.measure(point[0], point[1], channel)
    stats["actions"] += 1
    if resp.get("accepted"):
        mr = resp.get("measure_result")
        if mr == "direction":
            return mr, float(resp.get("svd_deg"))
        return mr, None
    stats["errors"].append(f"measure({point},{channel}) 未被接受: {resp}")
    return None, None


def try_clear_at(client, point, channel, stats):
    """在 point 处清除 channel, 返回是否成功."""
    resp = client.clear(point[0], point[1], channel)
    stats["actions"] += 1
    if resp.get("accepted"):
        if resp.get("clear_result") == "success":
            stats["cleared"] += 1
            stats["channels_cleared"].add(channel)
            return True
        return False
    stats["errors"].append(f"clear({point},{channel}) 未被接受: {resp}")
    return False


def run(robot_id, base_url, log_path):
    client = RobotClient(robot_id, base_url=base_url, log_path=log_path)

    print("=" * 60)
    print("B 题问题3 终版v2(自适应跳过+早终止+2-opt)")
    print(f"接口地址: {base_url}")
    print(f"robot_id: {robot_id}")
    print(f"共享站数: {len(PROBE_POINTS)} (中心 + 8 环站, 半径 {RING_RADIUS}m)")
    print(f"交会角阈值: {MIN_CUT_ANGLE}°, 跳过阈值: {TRIANGULATE_MIN_ANGLE}°")
    print(f"不使用无信号早终止(防漏源), 现实时间余量: {REAL_TIME_RESERVE}s")
    print("=" * 60)

    stats = {
        "cleared": 0,
        "channels_with_signal": set(),
        "channels_cleared": set(),
        "actions": 0,
        "errors": [],
        "skipped_channels": 0,
    }

    # 步骤1: /enter
    print("\n--- /enter ---")
    resp = client.enter()
    stats["actions"] += 1
    if not resp.get("accepted"):
        stats["errors"].append(f"/enter 失败: {json.dumps(resp, ensure_ascii=False)}")
        finalize(stats, client, log_path)
        return 1
    remaining_real = resp.get("remaining_real_duration_s", 1200)
    print(f"  本局可用现实时间: {remaining_real} 秒")

    # 步骤2: 自适应九站扫描(不使用无信号早终止, 必须扫完全部9站)
    # observations: channel -> [(point, bearing), ...]
    # triangulated: channel -> True/False (已够观测, 后续站跳过)
    observations = {}
    triangulated = set()  # 已满足跳过条件的频道

    for station_index, point in enumerate(PROBE_POINTS):
        channels = list(range(1, 21)) if station_index % 2 == 0 else list(range(20, 0, -1))
        print(f"\n--- 站 {station_index} ({point[0]:.0f},{point[1]:.0f}) ---")
        scanned_this_station = 0
        for ch in channels:
            # 现实时间余量检查
            if client.remaining_real_duration_s is not None and \
               client.remaining_real_duration_s <= REAL_TIME_RESERVE:
                print(f"  现实时间余量不足, 提前结束扫描")
                break

            # 自适应跳过: 已三角化的频道跳过(安全, 已有2次direction)
            if ch in triangulated:
                stats["skipped_channels"] += 1
                continue

            mr, svd = measure_at(client, point, ch, stats)
            scanned_this_station += 1

            if mr == "direction":
                observations.setdefault(ch, []).append((point, svd))
                stats["channels_with_signal"].add(ch)
                print(f"  频道{ch}: direction svd={svd} (观测数={len(observations[ch])})")

                # 检查是否已满足三角化条件
                if len(observations[ch]) >= 2:
                    max_ang = max_intersection_angle(observations[ch])
                    if max_ang >= TRIANGULATE_MIN_ANGLE:
                        triangulated.add(ch)
                        print(f"    -> 交会角={max_ang:.0f}°>={TRIANGULATE_MIN_ANGLE}°, "
                              f"后续站跳过该频道")

            elif mr == "near":
                if try_clear_at(client, point, ch, stats):
                    print(f"  频道{ch}: near, 清除成功")

            # 不做无信号早终止! 某频道可能在前几站无信号但在后几站有信号

        print(f"  本站扫描: {scanned_this_station} 频道, "
              f"跳过(三角化): {stats['skipped_channels']}")
        if client.remaining_real_duration_s is not None and \
           client.remaining_real_duration_s <= REAL_TIME_RESERVE:
            break

    print(f"\n发现信号频道: {sorted(stats['channels_with_signal'])}")
    print(f"near 直接清除: {sorted(stats['channels_cleared'])}")
    print(f"三角化跳过次数: {stats['skipped_channels']}")

    # 步骤3: 对只有一次 direction 的频道用 Q_± 补测一次
    for ch, obs in list(observations.items()):
        if ch in stats["channels_cleared"]:
            continue
        if len(obs) != 1:
            continue
        if client.remaining_real_duration_s is not None and \
           client.remaining_real_duration_s <= REAL_TIME_RESERVE:
            break
        first_point, first_bearing = obs[0]
        current_pos = (0.0, 0.0)
        q_pt = pick_q_pm_point(first_point, first_bearing, current_pos)
        if math.hypot(q_pt[0], q_pt[1]) > TARGET_RADIUS:
            scale = TARGET_RADIUS * 0.95 / max(1e-9, math.hypot(q_pt[0], q_pt[1]))
            q_pt = (q_pt[0] * scale, q_pt[1] * scale)
        print(f"\n--- 频道{ch} 补测点 Q=({q_pt[0]:.0f},{q_pt[1]:.0f}) ---")
        mr, svd = measure_at(client, q_pt, ch, stats)
        if mr == "direction":
            obs.append((q_pt, svd))
            print(f"  频道{ch}: 补测 direction svd={svd}")
        elif mr == "near":
            if try_clear_at(client, q_pt, ch, stats):
                print(f"  频道{ch}: 补测 near, 清除成功")

    # 步骤4: 中位数交会定位
    print(f"\n--- 中位数交会点(交会角>{MIN_CUT_ANGLE}°) ---")
    targets = []
    failed_channels = []
    for ch in stats["channels_with_signal"]:
        if ch in stats["channels_cleared"]:
            continue
        obs = observations.get(ch, [])
        if len(obs) < 2:
            continue
        inter, ang = median_intersection(obs)
        if inter is None:
            inter, ang = best_intersection(obs)
            if inter is None:
                print(f"  频道{ch}: 无有效交会, 跳过")
                continue
        d_origin = math.hypot(inter[0], inter[1])
        if d_origin > MAX_CLEAR_DIST_FROM_ORIGIN:
            print(f"  频道{ch}: 交会点({inter[0]:.0f},{inter[1]:.0f}) 越出圆域, 跳过")
            continue
        targets.append((ch, inter[0], inter[1], ang))
        print(f"  频道{ch}: ({inter[0]:.0f},{inter[1]:.0f}) 交会角={ang:.0f}° "
              f"观测数={len(obs)}")

    targets.sort(key=lambda t: -t[3])

    # 步骤5: 2-opt 优化的清除路径
    print(f"\n--- 2-opt 优化清除路径({len(targets)} 个目标) ---")
    current_pos = (0.0, 0.0)
    raw_order = nearest_neighbor_order(current_pos,
                                       [(t[0], t[1], t[2]) for t in targets])
    optimized = two_opt(raw_order, current_pos)

    raw_dist = sum(math.hypot(optimized[i][1] - (optimized[i-1][1] if i > 0 else 0),
                               optimized[i][2] - (optimized[i-1][2] if i > 0 else 0))
                    for i in range(len(optimized)))
    print(f"  优化后路径总距离: {raw_dist:.0f}m")

    for ch, x, y in optimized:
        if client.remaining_real_duration_s is not None and \
           client.remaining_real_duration_s <= REAL_TIME_RESERVE:
            print(f"  现实时间余量不足, 提前结束清除")
            break
        d = math.hypot(x - current_pos[0], y - current_pos[1])
        print(f"  频道{ch}: ({x:.0f},{y:.0f}) 距当前位置 {d:.0f}m")

        # 先在交会点measure确认, 再clear
        mr, svd = measure_at(client, (x, y), ch, stats)
        if mr == "near":
            # 确认在源附近, 直接清除
            if try_clear_at(client, (x, y), ch, stats):
                print(f"    near确认, 清除成功!")
                current_pos = (x, y)
                continue
        elif mr == "direction":
            # 交会点不在5m内但能看到源, 加入新观测重新定位
            observations.setdefault(ch, []).append(((x, y), svd))
            inter, ang = median_intersection(observations[ch])
            if inter is not None and \
               math.hypot(inter[0], inter[1]) <= MAX_CLEAR_DIST_FROM_ORIGIN:
                # 在新交会点再确认
                mr2, _ = measure_at(client, inter, ch, stats)
                if mr2 == "near":
                    if try_clear_at(client, inter, ch, stats):
                        print(f"    重新定位后清除成功!")
                        current_pos = inter
                        continue
                elif mr2 == "direction":
                    if try_clear_at(client, inter, ch, stats):
                        print(f"    重新定位后清除成功!")
                        current_pos = inter
                        continue

        # 直接清除尝试(可能交会点精度足够)
        if try_clear_at(client, (x, y), ch, stats):
            print(f"    清除成功!")
            current_pos = (x, y)
        else:
            print(f"    清除失败, 加入精扫队列")
            failed_channels.append((ch, x, y, observations.get(ch, [])))

    # 步骤6: 精扫失败频道
    if failed_channels:
        print(f"\n--- 对 {len(failed_channels)} 个失败频道精扫 ---")
        for ch, fx, fy, obs in failed_channels:
            if client.remaining_real_duration_s is not None and \
               client.remaining_real_duration_s <= REAL_TIME_RESERVE:
                print(f"  现实时间余量不足, 停止精扫")
                break
            if ch in stats["channels_cleared"]:
                continue
            print(f"  频道{ch}: 在 ({fx:.0f},{fy:.0f}) 附近 30m 精扫")
            if refine_clear(client, (fx, fy), ch, stats,
                           search_radius=30.0, step=10.0):
                print(f"    精扫清除成功!")
            else:
                if refine_clear(client, (fx, fy), ch, stats,
                               search_radius=50.0, step=15.0):
                    print(f"    扩大精扫清除成功!")
                else:
                    print(f"    精扫仍失败")

    # 步骤7: /exit
    print("\n--- /exit ---")
    resp = client.exit()
    stats["actions"] += 1

    # 汇总
    print("\n" + "=" * 60)
    print("问题3 终版v2演练汇总")
    print("=" * 60)
    print(f"总动作数: {stats['actions']}")
    print(f"有信号频道: {sorted(stats['channels_with_signal'])}")
    print(f"清除干扰源数: {stats['cleared']}")
    print(f"清除的频道: {sorted(stats['channels_cleared'])}")
    print(f"三角化跳过次数: {stats['skipped_channels']}")
    print(f"虚拟时间: {client.current_virtual_time:.2f} 秒")
    if stats["errors"]:
        print(f"错误数: {len(stats['errors'])}")
        for err in stats["errors"][:10]:
            print(f"  {err}")

    finalize(stats, client, log_path)
    return 0


def finalize(stats, client, log_path):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    summary_path = os.path.join(script_dir, "search_q3_final_summary.json")
    summary = {
        "cleared": stats["cleared"],
        "channels_with_signal": sorted(list(stats["channels_with_signal"])),
        "channels_cleared": sorted(list(stats["channels_cleared"])),
        "actions": stats["actions"],
        "virtual_time_s": client.current_virtual_time,
        "skipped_triangulated": stats.get("skipped_channels", 0),
        "errors": stats["errors"],
    }
    with open(summary_path, "w", encoding="utf-8") as fout:
        json.dump(summary, fout, ensure_ascii=False, indent=2)
    print(f"汇总已保存: {summary_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="B 题问题3 终版v2搜索算法")
    parser.add_argument("robot_id", help="参赛队号")
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument("--log", default="search_q3_final_log.jsonl")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(script_dir, args.log)
    if os.path.exists(log_path):
        os.remove(log_path)

    sys.exit(run(args.robot_id, args.base_url, log_path))
