# -*- coding: utf-8 -*-
"""
B 题问题4 终版搜索算法 v3(批量边缘点探测 + 沿射线补测)

核心优化(v3):
1. 批量边缘点探测: 遍历8个边缘点, 每个点检查全部C类频道
   (替代v2的"每频道遍历8个边缘点", 省约80%移动距离)
2. B类沿射线补测: Q_±失败后沿射线方向逐步补测(定向源扇区保持性)
3. 自适应频道跳过 + 无信号早终止(复用Q3优化)
4. 精扫上限8次测量(防止无限精扫)
5. 2-opt路径优化
"""
import argparse
import json
import math
import os
import sys

from robot_client import RobotClient
from search_q3_final import (
    PROBE_POINTS, angle_diff, triangulate, best_intersection,
    median_intersection, max_intersection_angle,
    nearest_neighbor_order, two_opt,
    pick_q_pm_point, refine_clear,
    measure_at, try_clear_at, finalize,
    TARGET_RADIUS, MIN_CUT_ANGLE, REAL_TIME_RESERVE,
    MAX_CLEAR_DIST_FROM_ORIGIN, Q_FORWARD, Q_LATERAL,
    TRIANGULATE_MIN_ANGLE, NOSIGNAL_EARLY_STOP,
    move_along_bearing,
)

# 边缘探测点: 半径 1700m, 8 个方向(与环站错开 22.5°)
EDGE_RADIUS = 1700.0
EDGE_POINTS = [
    (EDGE_RADIUS * math.cos(math.radians(22.5 + k * 45)),
     EDGE_RADIUS * math.sin(math.radians(22.5 + k * 45)))
    for k in range(8)
]

# 中心区补测点: 半径 300m, 4 个方向
CENTER_PROBE_RADIUS = 300.0
CENTER_PROBE_POINTS = [
    (CENTER_PROBE_RADIUS * math.cos(math.radians(k * 90)),
     CENTER_PROBE_RADIUS * math.sin(math.radians(k * 90)))
    for k in range(4)
]

# 沿射线补测距离列表(从小到大)
ALONG_BEARING_DISTANCES = [300.0, 600.0, 1000.0, 1400.0]


def run(robot_id, base_url, log_path):
    client = RobotClient(robot_id, base_url=base_url, log_path=log_path)

    print("=" * 60)
    print("B 题问题4 终版v3(批量边缘点+沿射线补测)")
    print(f"接口地址: {base_url}")
    print(f"robot_id: {robot_id}")
    print(f"共享站数: {len(PROBE_POINTS)}, 边缘探测点: {len(EDGE_POINTS)}")
    print("=" * 60)

    stats = {
        "cleared": 0,
        "channels_with_signal": set(),
        "channels_cleared": set(),
        "actions": 0,
        "errors": [],
        "skipped_channels": 0,
        "skipped_nosignal": 0,
        "directional_channels": [],
        "a_class": 0,
        "b_class": 0,
        "c_class": 0,
        "d_class": 0,
    }

    # 步骤1: /enter
    print("\n--- /enter ---")
    resp = client.enter()
    stats["actions"] += 1
    if not resp.get("accepted"):
        stats["errors"].append(f"/enter 失败: {json.dumps(resp, ensure_ascii=False)}")
        finalize(stats, client, log_path)
        return 1
    print(f"  本局可用现实时间: {resp.get('remaining_real_duration_s')} 秒")

    # 步骤2: 自适应九站扫描
    evidence = {}
    observations = {}  # channel -> [(point, bearing), ...] (direction only)
    nosignal_count = {}
    triangulated = set()
    skipped_nosignal = set()

    for station_index, point in enumerate(PROBE_POINTS):
        channels = list(range(1, 21)) if station_index % 2 == 0 else list(range(20, 0, -1))
        print(f"\n--- 站 {station_index} ({point[0]:.0f},{point[1]:.0f}) ---")
        scanned = 0
        for ch in channels:
            if client.remaining_real_duration_s is not None and \
               client.remaining_real_duration_s <= REAL_TIME_RESERVE:
                print(f"  现实时间余量不足, 提前结束扫描")
                break
            if ch in triangulated:
                stats["skipped_channels"] += 1
                continue
            if ch in skipped_nosignal:
                stats["skipped_nosignal"] += 1
                continue

            mr, svd = measure_at(client, point, ch, stats)
            scanned += 1
            evidence.setdefault(ch, []).append((point, mr, svd))

            if mr == "direction":
                observations.setdefault(ch, []).append((point, svd))
                stats["channels_with_signal"].add(ch)
                nosignal_count[ch] = 0
                print(f"  频道{ch}: direction svd={svd} (观测数={len(observations[ch])})")
                if len(observations[ch]) >= 2:
                    max_ang = max_intersection_angle(observations[ch])
                    if max_ang >= TRIANGULATE_MIN_ANGLE:
                        triangulated.add(ch)
                        print(f"    -> 交会角={max_ang:.0f}°, 跳过")
            elif mr == "near":
                if try_clear_at(client, point, ch, stats):
                    print(f"  频道{ch}: near, 清除成功")
                    stats["d_class"] += 1
            elif mr == "no_signal":
                nosignal_count[ch] = nosignal_count.get(ch, 0) + 1
                if nosignal_count[ch] >= NOSIGNAL_EARLY_STOP and \
                   ch not in stats["channels_with_signal"]:
                    skipped_nosignal.add(ch)
                    print(f"  频道{ch}: 连续{nosignal_count[ch]}站无信号, 跳过")

        print(f"  本站扫描: {scanned} 频道, 跳过(三角化): {stats['skipped_channels']}")
        if client.remaining_real_duration_s is not None and \
           client.remaining_real_duration_s <= REAL_TIME_RESERVE:
            break

    print(f"\n发现信号频道: {sorted(stats['channels_with_signal'])}")
    print(f"near直接清除(D类): {stats['d_class']} 个")

    # 步骤3: A类处理(>=2次direction)
    targets = []
    failed_channels = []

    for ch in sorted(stats["channels_with_signal"]):
        if ch in stats["channels_cleared"]:
            continue
        obs = observations.get(ch, [])
        if len(obs) >= 2:
            stats["a_class"] += 1
            inter, ang = median_intersection(obs)
            if inter is None:
                inter, ang = best_intersection(obs)
                if inter is None:
                    print(f"  频道{ch}: A类但无有效交会, 跳过")
                    continue
            if math.hypot(inter[0], inter[1]) > MAX_CLEAR_DIST_FROM_ORIGIN:
                print(f"  频道{ch}: A类交会点越出圆域, 跳过")
                continue
            targets.append((ch, inter[0], inter[1], ang))
            print(f"  频道{ch}: A类, 交会({inter[0]:.0f},{inter[1]:.0f}) 角={ang:.0f}°")

    # 步骤4: B类处理(1次direction)
    for ch in sorted(stats["channels_with_signal"]):
        if ch in stats["channels_cleared"]:
            continue
        obs = observations.get(ch, [])
        if len(obs) != 1:
            continue
        stats["b_class"] += 1
        first_point, first_bearing = obs[0]

        if client.remaining_real_duration_s is not None and \
           client.remaining_real_duration_s <= REAL_TIME_RESERVE:
            break

        # 4a: 先试 Q_±
        q_pt = pick_q_pm_point(first_point, first_bearing, (0.0, 0.0))
        if math.hypot(q_pt[0], q_pt[1]) > TARGET_RADIUS:
            scale = TARGET_RADIUS * 0.95 / max(1e-9, math.hypot(q_pt[0], q_pt[1]))
            q_pt = (q_pt[0] * scale, q_pt[1] * scale)
        print(f"\n  频道{ch}: B类, Q_±补测=({q_pt[0]:.0f},{q_pt[1]:.0f})")
        mr, svd = measure_at(client, q_pt, ch, stats)
        if mr == "direction":
            obs.append((q_pt, svd))
            inter, ang = best_intersection(obs)
            if inter is not None and math.hypot(inter[0], inter[1]) <= MAX_CLEAR_DIST_FROM_ORIGIN:
                targets.append((ch, inter[0], inter[1], ang))
                print(f"    Q_±成功, 交会({inter[0]:.0f},{inter[1]:.0f}) 角={ang:.0f}°")
                continue
        elif mr == "near":
            if try_clear_at(client, q_pt, ch, stats):
                print(f"    Q_± near, 清除成功")
                continue

        # 4b: Q_±失败, 沿射线补测(定向源扇区保持性)
        print(f"  频道{ch}: Q_±无信号, 沿射线补测")
        stats["directional_channels"].append(ch)
        found_second = False
        for dist in ALONG_BEARING_DISTANCES:
            if client.remaining_real_duration_s is not None and \
               client.remaining_real_duration_s <= REAL_TIME_RESERVE:
                break
            pt = move_along_bearing(first_point[0], first_point[1],
                                    first_bearing, dist)
            if math.hypot(pt[0], pt[1]) > TARGET_RADIUS:
                scale = TARGET_RADIUS * 0.95 / max(1e-9, math.hypot(pt[0], pt[1]))
                pt = (pt[0] * scale, pt[1] * scale)
            mr2, svd2 = measure_at(client, pt, ch, stats)
            if mr2 == "direction":
                obs.append((pt, svd2))
                inter, ang = best_intersection(obs)
                if inter is not None and \
                   math.hypot(inter[0], inter[1]) <= MAX_CLEAR_DIST_FROM_ORIGIN:
                    targets.append((ch, inter[0], inter[1], ang))
                    print(f"    沿射线{dist}m成功, 角={ang:.0f}°")
                    found_second = True
                    break
            elif mr2 == "near":
                if try_clear_at(client, pt, ch, stats):
                    found_second = True
                    break
        if found_second:
            continue

        # 4c: 沿射线也失败, 垂直方向
        for perp_dir in [(first_bearing + 90) % 360, (first_bearing - 90) % 360]:
            if client.remaining_real_duration_s is not None and \
               client.remaining_real_duration_s <= REAL_TIME_RESERVE:
                break
            for dist in [400.0, 800.0]:
                pt = move_along_bearing(first_point[0], first_point[1],
                                        perp_dir, dist)
                if math.hypot(pt[0], pt[1]) > TARGET_RADIUS:
                    continue
                mr3, svd3 = measure_at(client, pt, ch, stats)
                if mr3 == "direction":
                    obs.append((pt, svd3))
                    inter, ang = best_intersection(obs)
                    if inter is not None and \
                       math.hypot(inter[0], inter[1]) <= MAX_CLEAR_DIST_FROM_ORIGIN:
                        targets.append((ch, inter[0], inter[1], ang))
                        print(f"    垂直{perp_dir:.0f}°+{dist}m成功")
                        found_second = True
                        break
                elif mr3 == "near":
                    if try_clear_at(client, pt, ch, stats):
                        found_second = True
                        break
            if found_second:
                break

    # 步骤5: C类批量边缘点探测
    # 收集所有C类频道(0次direction且未清除)
    c_channels = []
    for ch in range(1, 21):
        if ch in stats["channels_cleared"] or ch in stats["channels_with_signal"]:
            continue
        ev = evidence.get(ch, [])
        has_nosignal = any(r == "no_signal" for (_, r, _) in ev)
        if has_nosignal or len(ev) == 0:
            c_channels.append(ch)

    if c_channels:
        print(f"\n--- C类批量探测: {len(c_channels)}个频道 ---")
        stats["c_class"] = len(c_channels)

        # 5a: 批量边缘点探测 - 每个边缘点检查所有C类频道
        edge_hits = {}  # channel -> [(edge_point, svd), ...]
        for ep_idx, ep in enumerate(EDGE_POINTS):
            if client.remaining_real_duration_s is not None and \
               client.remaining_real_duration_s <= REAL_TIME_RESERVE:
                print(f"  现实时间余量不足, 停止边缘探测")
                break
            if not c_channels:
                break
            print(f"  边缘点{ep_idx} ({ep[0]:.0f},{ep[1]:.0f}): "
                  f"检查{len(c_channels)}个C类频道")
            # 在此边缘点检查所有未清除的C类频道
            remaining_c = []
            for ch in c_channels:
                if ch in stats["channels_cleared"]:
                    continue
                mr, svd = measure_at(client, ep, ch, stats)
                if mr == "direction":
                    observations.setdefault(ch, []).append((ep, svd))
                    stats["channels_with_signal"].add(ch)
                    stats["directional_channels"].append(ch)
                    edge_hits.setdefault(ch, []).append((ep, svd))
                    print(f"    频道{ch}: direction svd={svd}")
                    # 不从c_channels移除, 下个边缘点可能也命中
                elif mr == "near":
                    if try_clear_at(client, ep, ch, stats):
                        print(f"    频道{ch}: near, 清除成功")
                remaining_c.append(ch)
            c_channels = remaining_c

        # 5b: 对边缘点命中的频道做沿射线补测获取第二次观测
        print(f"\n--- 边缘点命中频道: {sorted(edge_hits.keys())} ---")
        for ch, hits in edge_hits.items():
            if ch in stats["channels_cleared"]:
                continue
            if len(hits) >= 2:
                # 已有2次direction, 直接定位
                obs = observations[ch]
                inter, ang = median_intersection(obs)
                if inter is not None and \
                   math.hypot(inter[0], inter[1]) <= MAX_CLEAR_DIST_FROM_ORIGIN:
                    targets.append((ch, inter[0], inter[1], ang))
                    print(f"  频道{ch}: 2次边缘命中, 交会({inter[0]:.0f},{inter[1]:.0f})")
                    continue

            if len(hits) == 1:
                # 只命中1次, 沿射线补测
                ep, svd = hits[0]
                found = False
                for dist in ALONG_BEARING_DISTANCES:
                    if client.remaining_real_duration_s is not None and \
                       client.remaining_real_duration_s <= REAL_TIME_RESERVE:
                        break
                    pt = move_along_bearing(ep[0], ep[1], svd, dist)
                    if math.hypot(pt[0], pt[1]) > TARGET_RADIUS:
                        scale = TARGET_RADIUS * 0.95 / max(1e-9, math.hypot(pt[0], pt[1]))
                        pt = (pt[0] * scale, pt[1] * scale)
                    mr2, svd2 = measure_at(client, pt, ch, stats)
                    if mr2 == "direction":
                        observations[ch].append((pt, svd2))
                        inter, ang = best_intersection(observations[ch])
                        if inter is not None and \
                           math.hypot(inter[0], inter[1]) <= MAX_CLEAR_DIST_FROM_ORIGIN:
                            targets.append((ch, inter[0], inter[1], ang))
                            print(f"  频道{ch}: 沿射线{dist}m, 交会({inter[0]:.0f},{inter[1]:.0f})")
                            found = True
                            break
                    elif mr2 == "near":
                        if try_clear_at(client, pt, ch, stats):
                            found = True
                            break

        # 5c: 边缘点全失败的频道, 尝试中心区点(批量)
        remaining_c = [ch for ch in c_channels
                       if ch not in stats["channels_cleared"]
                       and ch not in edge_hits
                       and ch not in stats["channels_with_signal"]]
        if remaining_c:
            print(f"\n--- 边缘全失败, 中心区探测{len(remaining_c)}个频道 ---")
            for cp in CENTER_PROBE_POINTS:
                if client.remaining_real_duration_s is not None and \
                   client.remaining_real_duration_s <= REAL_TIME_RESERVE:
                    break
                if not remaining_c:
                    break
                print(f"  中心区点({cp[0]:.0f},{cp[1]:.0f}): "
                      f"检查{len(remaining_c)}个频道")
                still_remaining = []
                for ch in remaining_c:
                    if ch in stats["channels_cleared"]:
                        continue
                    mr, svd = measure_at(client, cp, ch, stats)
                    if mr == "direction":
                        observations.setdefault(ch, []).append((cp, svd))
                        stats["channels_with_signal"].add(ch)
                        stats["directional_channels"].append(ch)
                        print(f"    频道{ch}: direction svd={svd}")
                        # 沿射线补测
                        for dist in ALONG_BEARING_DISTANCES:
                            if client.remaining_real_duration_s is not None and \
                               client.remaining_real_duration_s <= REAL_TIME_RESERVE:
                                break
                            pt = move_along_bearing(cp[0], cp[1], svd, dist)
                            if math.hypot(pt[0], pt[1]) > TARGET_RADIUS:
                                scale = TARGET_RADIUS * 0.95 / max(1e-9, math.hypot(pt[0], pt[1]))
                                pt = (pt[0] * scale, pt[1] * scale)
                            mr2, svd2 = measure_at(client, pt, ch, stats)
                            if mr2 == "direction":
                                observations[ch].append((pt, svd2))
                                inter, ang = best_intersection(observations[ch])
                                if inter is not None and \
                                   math.hypot(inter[0], inter[1]) <= MAX_CLEAR_DIST_FROM_ORIGIN:
                                    targets.append((ch, inter[0], inter[1], ang))
                                    print(f"      沿射线{dist}m, 交会({inter[0]:.0f},{inter[1]:.0f})")
                                    break
                            elif mr2 == "near":
                                try_clear_at(client, pt, ch, stats)
                                break
                    elif mr == "near":
                        try_clear_at(client, cp, ch, stats)
                    still_remaining.append(ch)
                remaining_c = still_remaining

            if remaining_c:
                print(f"  中心区也全失败, 跳过: {remaining_c}")

    # 步骤6: 按交会角排序 + 2-opt路径优化
    targets.sort(key=lambda t: -t[3])
    print(f"\n--- 2-opt优化清除路径({len(targets)}个目标) ---")
    current_pos = (0.0, 0.0)
    raw_order = nearest_neighbor_order(current_pos,
                                       [(t[0], t[1], t[2]) for t in targets])
    optimized = two_opt(raw_order, current_pos)

    for ch, x, y in optimized:
        if client.remaining_real_duration_s is not None and \
           client.remaining_real_duration_s <= REAL_TIME_RESERVE:
            print(f"  现实时间余量不足, 提前结束")
            break
        d = math.hypot(x - current_pos[0], y - current_pos[1])
        print(f"  频道{ch}: ({x:.0f},{y:.0f}) 距当前位置 {d:.0f}m")
        if try_clear_at(client, (x, y), ch, stats):
            print(f"    清除成功!")
            current_pos = (x, y)
        else:
            print(f"    清除失败, 加入精扫队列")
            failed_channels.append((ch, x, y, observations.get(ch, [])))

    # 步骤7: 精扫失败频道(上限8次测量)
    if failed_channels:
        print(f"\n--- 对{len(failed_channels)}个失败频道精扫 ---")
        for ch, fx, fy, obs in failed_channels:
            if client.remaining_real_duration_s is not None and \
               client.remaining_real_duration_s <= REAL_TIME_RESERVE:
                break
            if ch in stats["channels_cleared"]:
                continue
            print(f"  频道{ch}: ({fx:.0f},{fy:.0f})附近30m精扫")
            if refine_clear(client, (fx, fy), ch, stats, 30.0, 10.0,
                           max_measurements=8):
                print(f"    精扫清除成功!")
            elif refine_clear(client, (fx, fy), ch, stats, 50.0, 15.0,
                              max_measurements=8):
                print(f"    扩大精扫清除成功!")
            else:
                print(f"    精扫仍失败")

    # 步骤8: /exit
    print("\n--- /exit ---")
    client.exit()
    stats["actions"] += 1

    # 汇总
    print("\n" + "=" * 60)
    print("问题4 终版v3演练汇总")
    print("=" * 60)
    print(f"总动作数: {stats['actions']}")
    print(f"有信号频道: {sorted(stats['channels_with_signal'])}")
    print(f"清除干扰源数: {stats['cleared']}")
    print(f"清除的频道: {sorted(stats['channels_cleared'])}")
    print(f"A类: {stats['a_class']}, B类: {stats['b_class']}, "
          f"C类: {stats['c_class']}, D类: {stats['d_class']}")
    print(f"判定为定向源频道: {stats['directional_channels']}")
    print(f"虚拟时间: {client.current_virtual_time:.2f} 秒")
    if stats["errors"]:
        print(f"错误数: {len(stats['errors'])}")

    # 保存专属 summary
    script_dir = os.path.dirname(os.path.abspath(__file__))
    summary_path = os.path.join(script_dir, "search_q4_final_summary.json")
    summary = {
        "cleared": stats["cleared"],
        "channels_with_signal": sorted(list(stats["channels_with_signal"])),
        "channels_cleared": sorted(list(stats["channels_cleared"])),
        "directional_channels": stats["directional_channels"],
        "a_class": stats["a_class"],
        "b_class": stats["b_class"],
        "c_class": stats["c_class"],
        "d_class": stats["d_class"],
        "actions": stats["actions"],
        "virtual_time_s": client.current_virtual_time,
        "errors": stats["errors"],
    }
    with open(summary_path, "w", encoding="utf-8") as fout:
        json.dump(summary, fout, ensure_ascii=False, indent=2)
    print(f"汇总已保存: {summary_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="B 题问题4 终版v3搜索算法")
    parser.add_argument("robot_id", help="参赛队号")
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument("--log", default="search_q4_final_log.jsonl")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(script_dir, args.log)
    if os.path.exists(log_path):
        os.remove(log_path)

    run(args.robot_id, args.base_url, log_path)
