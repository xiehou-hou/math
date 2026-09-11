# -*- coding: utf-8 -*-
"""
B 题问题 3 演练测试:交会定位法 v5(时间优化版)

v4→v5 核心改进:
1. 时间预算:全局动作上限45,虚拟时间硬上限1400秒(留100秒余量)
2. 就近清除:移动到交会点时,按距离排序,先清近的
3. 距离阈值:交会点距原点>1000米的频道,延后处理(移动耗时=200秒)
4. 网格搜索取消:v4成功率极低(1/6),改为交会失败直接放弃该检测点对
5. 第三检测点更近:300米(减少移动耗时)
6. 路径优化:清除时从当前位置出发,选最近的已定位目标
7. 无解型频道:B、C均无信号直接放弃,不再尝试逐步逼近
"""
import argparse
import json
import math
import sys
import os

from robot_client import RobotClient

MAX_ACTIONS = 45
MAX_VIRTUAL_TIME = 1300.0  # 虚拟时间硬上限(收紧)
LOG_SIZE_LIMIT = 2 * 1024 * 1024

POINT_A = (0.0, 0.0)
SECOND_POINT_DIST = 400.0  # 第二检测点距离(减少移动)
THIRD_POINT_DIST = 300.0
MAX_CLEAR_DIST = 1000.0  # 超过此距离的交会点延后处理


def triangulate(p1, theta1_deg, p2, theta2_deg):
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


def angle_diff(a, b):
    d = abs(a - b) % 360
    return 360 - d if d > 180 else d


def move_along_bearing(x, y, bearing_deg, dist_m):
    rad = math.radians(bearing_deg)
    return x + dist_m * math.cos(rad), y + dist_m * math.sin(rad)


def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def pick_second_point(bearing_deg, d=SECOND_POINT_DIST):
    p1 = move_along_bearing(0, 0, (bearing_deg + 90) % 360, d)
    p2 = move_along_bearing(0, 0, (bearing_deg - 90) % 360, d)
    return p1 if p1[0] >= p2[0] else p2


def measure_at(client, point, channel, stats):
    resp = client.measure(point[0], point[1], channel)
    stats["actions"] += 1
    if resp.get("accepted"):
        mr = resp.get("measure_result")
        return (mr, resp.get("svd_deg")) if mr == "direction" else (mr, None)
    stats["errors"].append(f"measure({point},{channel}) 未被接受")
    return None, None


def try_clear_at(client, point, channel, stats):
    resp = client.clear(point[0], point[1], channel)
    stats["actions"] += 1
    if resp.get("accepted"):
        if resp.get("clear_result") == "success":
            stats["cleared"] += 1
            stats["channels_cleared"].add(channel)
            return True
        return False
    stats["errors"].append(f"clear({point},{channel}) 未被接受")
    return False


def compute_clear_targets(bearings_all, points):
    """计算所有频道的交会点,返回 [(channel, inter_point, distance_from_origin), ...]
    按距原点从近到远排序"""
    targets = []
    point_names = list(bearings_all.keys())
    # 收集所有频道号(并集)
    all_channels = set()
    for pn in point_names:
        all_channels.update(bearings_all[pn].keys())
    for ch in all_channels:
        # 对每个频道,找所有检测点对中交会角最大的
        best_inter = None
        best_diff = 0
        for i in range(len(point_names)):
            for j in range(i+1, len(point_names)):
                p1n, p2n = point_names[i], point_names[j]
                if ch not in bearings_all[p1n] or ch not in bearings_all[p2n]:
                    continue
                theta1 = bearings_all[p1n][ch]
                theta2 = bearings_all[p2n][ch]
                diff = angle_diff(theta1, theta2)
                if diff < 15:
                    # 中点策略
                    mid = ((points[p1n][0]+points[p2n][0])/2, (points[p1n][1]+points[p2n][1])/2)
                    if best_inter is None or diff > best_diff:
                        best_inter = mid
                        best_diff = diff
                    continue
                inter = triangulate(points[p1n], theta1, points[p2n], theta2)
                if inter and diff > best_diff:
                    best_inter = inter
                    best_diff = diff
        if best_inter:
            d = dist(best_inter, (0, 0))
            targets.append((ch, best_inter, d, best_diff))
    targets.sort(key=lambda t: t[2])  # 按距离从近到远
    return targets


def main():
    parser = argparse.ArgumentParser(description="B 题问题3 演练(交会定位法v5)")
    parser.add_argument("robot_id", help="参赛队号")
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument("--log", default="rehearsal_triangulate_v5_log.jsonl")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(script_dir, args.log)
    if os.path.exists(log_path):
        os.remove(log_path)

    client = RobotClient(args.robot_id, base_url=args.base_url, log_path=log_path)

    print("=" * 60)
    print("B 题问题3 演练(交会定位法 v5)")
    print(f"接口地址: {args.base_url}")
    print(f"robot_id: {args.robot_id}")
    print(f"改进:时间预算1400s+就近清除+距离阈值+网格取消")
    print("=" * 60)

    stats = {
        "cleared": 0,
        "channels_with_signal": set(),
        "channels_cleared": set(),
        "actions": 0,
        "errors": [],
    }

    # 步骤1: /enter
    print("\n--- /enter ---")
    resp = client.enter()
    stats["actions"] += 1
    if not resp.get("accepted"):
        stats["errors"].append(f"/enter 失败: {json.dumps(resp, ensure_ascii=False)}")
        finalize(stats, client, log_path)
        return 1

    # 步骤2: A 点扫描频道 1-20
    print(f"\n--- A={POINT_A} 扫描频道 1-20 ---")
    bearings_A = {}
    for ch in range(1, 21):
        if stats["actions"] >= MAX_ACTIONS:
            break
        mr, svd = measure_at(client, POINT_A, ch, stats)
        if mr == "direction":
            bearings_A[ch] = svd
            stats["channels_with_signal"].add(ch)
            print(f"  频道{ch}: direction svd={svd}")
        elif mr == "near":
            if try_clear_at(client, POINT_A, ch, stats):
                print(f"  频道{ch}: near,清除成功")

    print(f"\nA 点有信号频道: {sorted(bearings_A.keys())}")
    if not bearings_A and not stats["channels_cleared"]:
        client.exit()
        stats["actions"] += 1
        finalize(stats, client, log_path)
        return 0

    # 步骤3: 动态选第二检测点,测有信号频道
    to_measure = sorted(set(bearings_A.keys()) - stats["channels_cleared"])
    avg_bearing = sum(bearings_A[ch] for ch in to_measure) / len(to_measure) if to_measure else 0
    second_point = pick_second_point(avg_bearing)
    print(f"\n--- 第二检测点(垂直于平均示向度{avg_bearing:.1f}°): ({second_point[0]:.0f},{second_point[1]:.0f}) ---")
    bearings_B = {}
    no_signal_at_B = set()
    for ch in to_measure:
        if stats["actions"] >= MAX_ACTIONS:
            break
        mr, svd = measure_at(client, second_point, ch, stats)
        if mr == "direction":
            bearings_B[ch] = svd
            print(f"  频道{ch}: direction svd={svd}")
        elif mr == "near":
            if try_clear_at(client, second_point, ch, stats):
                print(f"  频道{ch}: near,清除成功")
        elif mr == "no_signal":
            no_signal_at_B.add(ch)
            print(f"  频道{ch}: no_signal")

    # 步骤4: 计算交会点,按距离排序,就近清除
    points = {"A": POINT_A, "B": second_point}
    bearings_all = {"A": bearings_A, "B": bearings_B}

    targets = compute_clear_targets(bearings_all, points)
    print(f"\n--- 交会点(按距离排序) ---")
    for ch, inter, d, diff in targets:
        print(f"  频道{ch}: ({inter[0]:.0f},{inter[1]:.0f}) 距原点{d:.0f}m 交会角{diff:.0f}°")

    # 就近清除:从当前位置(第二检测点)出发,清除最近的已定位目标
    current_pos = second_point
    for ch, inter, d_from_origin, diff in targets:
        if stats["actions"] >= MAX_ACTIONS - 1:
            break
        if client.current_virtual_time >= MAX_VIRTUAL_TIME:
            print(f"  虚拟时间超{MAX_VIRTUAL_TIME}s,停止")
            break
        d_from_current = dist(inter, current_pos)
        # 跳过远距离且交会角极小的频道(移动耗时大,定位不准)
        if d_from_current > 1000 and diff < 20:
            print(f"  频道{ch}: 距当前位置{d_from_current:.0f}m>1000且交会角{diff:.0f}°<20°,跳过")
            continue
        print(f"  频道{ch}: ({inter[0]:.0f},{inter[1]:.0f}) 距当前位置{d_from_current:.0f}m 交会角{diff:.0f}°")
        if try_clear_at(client, inter, ch, stats):
            print(f"    清除成功!")
            current_pos = inter
        elif diff >= 15:
            # 交会角>=15°才做网格搜索,步长50米,最多2步
            print(f"    清除失败,在交会点附近搜索...")
            for dx, dy in [(-50, 0), (50, 0)]:
                if stats["actions"] >= MAX_ACTIONS or client.current_virtual_time >= MAX_VIRTUAL_TIME:
                    break
                alt = (inter[0] + dx, inter[1] + dy)
                if try_clear_at(client, alt, ch, stats):
                    print(f"      偏移({dx},{dy})清除成功!")
                    current_pos = alt
                    break
        else:
            print(f"    交会角{diff:.0f}°<15°,伪交会点,跳过网格搜索")

    # 步骤5: 对B无信号频道,用第三检测点(更近)
    # 同时对交会角极小(<15°)的频道,也在第三点重新测量
    # 预初始化,避免步骤6引用未定义变量
    bearings_C = {}
    third_point = None
    no_signal_channels = sorted(no_signal_at_B - stats["channels_cleared"])
    # 找交会角极小的频道(伪交会点),需要在第三点重新测
    small_angle_channels = set()
    for ch in bearings_A:
        if ch in bearings_B and ch not in stats["channels_cleared"]:
            diff = angle_diff(bearings_A[ch], bearings_B[ch])
            if diff < 15:
                small_angle_channels.add(ch)
    # 被跳过的远距离频道也加入第三点测量
    skipped_far = set()
    for ch in bearings_A:
        if ch in bearings_B and ch not in stats["channels_cleared"]:
            diff = angle_diff(bearings_A[ch], bearings_B[ch])
            if diff < 20:  # 被跳过的条件
                skipped_far.add(ch)
    needs_third_point = sorted(set(no_signal_channels) | set(small_angle_channels) | set(skipped_far) - stats["channels_cleared"])
    if needs_third_point and stats["actions"] < MAX_ACTIONS - 3 and client.current_virtual_time < MAX_VIRTUAL_TIME:
        print(f"\n--- 第三检测点处理B无信号+交会角极小频道 {needs_third_point} ---")
        third_point = pick_second_point(avg_bearing + 90, d=THIRD_POINT_DIST)
        print(f"  第三检测点: ({third_point[0]:.0f},{third_point[1]:.0f})")
        bearings_C = {}
        for ch in needs_third_point:
            if stats["actions"] >= MAX_ACTIONS or client.current_virtual_time >= MAX_VIRTUAL_TIME:
                break
            mr, svd = measure_at(client, third_point, ch, stats)
            if mr == "direction":
                bearings_C[ch] = svd
                print(f"  频道{ch}: direction svd={svd}")
            elif mr == "near":
                if try_clear_at(client, third_point, ch, stats):
                    print(f"  频道{ch}: near,清除成功")

        # 三点交会清除(A、C或B、C)
        points["C"] = third_point
        bearings_all["C"] = bearings_C
        remaining = sorted(set(needs_third_point) - stats["channels_cleared"])
        for ch in remaining:
            if ch not in bearings_C:
                print(f"  频道{ch}: B、C均无信号,放弃(无解型)")
                continue
            # 优先用A、C交会(交会角可能更大)
            best_inter = None
            best_diff = 0
            if ch in bearings_A:
                diff_ac = angle_diff(bearings_A[ch], bearings_C[ch])
                if diff_ac >= 15:
                    inter = triangulate(POINT_A, bearings_A[ch], third_point, bearings_C[ch])
                    if inter:
                        best_inter = inter
                        best_diff = diff_ac
            # 如果A、C交会角也小,用B、C交会
            if best_inter is None and ch in bearings_B:
                diff_bc = angle_diff(bearings_B[ch], bearings_C[ch])
                if diff_bc >= 15:
                    inter = triangulate(second_point, bearings_B[ch], third_point, bearings_C[ch])
                    if inter and diff_bc > best_diff:
                        best_inter = inter
                        best_diff = diff_bc
            if best_inter:
                d = dist(best_inter, current_pos)
                if d < 1000 and stats["actions"] < MAX_ACTIONS and client.current_virtual_time < MAX_VIRTUAL_TIME:
                    print(f"  频道{ch}: 交会({best_inter[0]:.0f},{best_inter[1]:.0f}) 距当前位置{d:.0f}m 交会角{best_diff:.0f}°")
                    if try_clear_at(client, best_inter, ch, stats):
                        print(f"    清除成功!")
                        current_pos = best_inter
                    else:
                        print(f"    清除失败")
                else:
                    print(f"  频道{ch}: 交会点太远({d:.0f}m)或时间不足,跳过")

    # 步骤6: /exit(取消无效的中点策略)

    # 步骤7: /exit
    print("\n--- /exit ---")
    resp = client.exit()
    stats["actions"] += 1

    # 汇总
    print("\n" + "=" * 60)
    print("交会定位 v5 演练汇总")
    print("=" * 60)
    print(f"总动作数: {stats['actions']}")
    print(f"有信号频道: {sorted(stats['channels_with_signal'])}")
    print(f"清除干扰源数: {stats['cleared']}")
    print(f"清除的频道: {sorted(stats['channels_cleared'])}")
    print(f"虚拟时间: {client.current_virtual_time:.2f} 秒")
    if client.current_virtual_time > 1500:
        print(f"警告: 虚拟时间超过1500秒窗口!")
    if stats["errors"]:
        print(f"错误数: {len(stats['errors'])}")
        for err in stats["errors"][:10]:
            print(f"  {err}")

    log_size = os.path.getsize(log_path) if os.path.exists(log_path) else 0
    print(f"\n日志大小: {log_size} 字节 ({log_size/1024:.1f} KB)")
    if log_size > LOG_SIZE_LIMIT:
        print("警告: 日志超过 2MB!")

    finalize(stats, client, log_path)
    return 0


def finalize(stats, client, log_path):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    summary_path = os.path.join(script_dir, "rehearsal_triangulate_v5_summary.json")
    summary = {
        "cleared": stats["cleared"],
        "channels_with_signal": sorted(list(stats["channels_with_signal"])),
        "channels_cleared": sorted(list(stats["channels_cleared"])),
        "actions": stats["actions"],
        "virtual_time_s": client.current_virtual_time,
        "errors": stats["errors"],
    }
    with open(summary_path, "w", encoding="utf-8") as fout:
        json.dump(summary, fout, ensure_ascii=False, indent=2)
    print(f"汇总已保存: {summary_path}")


if __name__ == "__main__":
    sys.exit(main())
