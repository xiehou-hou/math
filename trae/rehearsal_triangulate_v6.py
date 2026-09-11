# -*- coding: utf-8 -*-
"""
B 题问题 3 演练测试:交会定位法 v6

v5→v6 核心改进:
1. 同频道多干扰源处理:示向度差异>60°时,可能有两个干扰源
   - 在A、B示向度方向各试清除一次
2. 优先级排序:先清交会角45°-90°的(最准),再清15°-45°的
3. 远距离目标跳过:距当前位置>800米直接放弃
4. 时间预算更紧:MAX_VIRTUAL_TIME=1350秒(留150秒余量)
5. A、B扫描只测有信号频道,减少动作数
6. 三点定位优化:第三点选在与A、B示向度都垂直的方向
7. 失败频道二次尝试:沿A示向度走500米,再测再交会
"""
import argparse
import json
import math
import sys
import os

from robot_client import RobotClient

MAX_ACTIONS = 42
MAX_VIRTUAL_TIME = 1350.0
LOG_SIZE_LIMIT = 2 * 1024 * 1024

POINT_A = (0.0, 0.0)
SECOND_POINT_DIST = 400.0
THIRD_POINT_DIST = 300.0
MAX_MOVE_DIST = 800.0  # 超过此距离放弃


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
    """计算所有频道的交会点,返回 [(channel, inter_point, distance_from_origin, diff), ...]
    按交会角从大到小排序(交会角大=定位准=优先清)"""
    targets = []
    point_names = list(bearings_all.keys())
    all_channels = set()
    for pn in point_names:
        all_channels.update(bearings_all[pn].keys())
    for ch in all_channels:
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
    # 按交会角从大到小排序(交会角大=定位准)
    targets.sort(key=lambda t: -t[3])
    return targets


def main():
    parser = argparse.ArgumentParser(description="B 题问题3 演练(交会定位法v6)")
    parser.add_argument("robot_id", help="参赛队号")
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument("--log", default="rehearsal_triangulate_v6_log.jsonl")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(script_dir, args.log)
    if os.path.exists(log_path):
        os.remove(log_path)

    client = RobotClient(args.robot_id, base_url=args.base_url, log_path=log_path)

    print("=" * 60)
    print("B 题问题3 演练(交会定位法 v6)")
    print(f"接口地址: {args.base_url}")
    print(f"robot_id: {args.robot_id}")
    print(f"改进:多干扰源处理+优先级排序+时间预算1350s")
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

    # 步骤3: 动态选第二检测点
    to_measure = sorted(set(bearings_A.keys()) - stats["channels_cleared"])
    avg_bearing = sum(bearings_A[ch] for ch in to_measure) / len(to_measure) if to_measure else 0
    second_point = pick_second_point(avg_bearing)
    print(f"\n--- 第二检测点(垂直于平均示向度{avg_bearing:.1f}°): ({second_point[0]:.0f},{second_point[1]:.0f}) ---")
    bearings_B = {}
    no_signal_at_B = set()
    multi_source_suspect = {}  # 频道 -> (theta_A, theta_B) 差异>60°,疑似多干扰源
    for ch in to_measure:
        if stats["actions"] >= MAX_ACTIONS:
            break
        mr, svd = measure_at(client, second_point, ch, stats)
        if mr == "direction":
            bearings_B[ch] = svd
            print(f"  频道{ch}: direction svd={svd}")
            # 检测示向度差异是否过大(疑似多干扰源)
            diff = angle_diff(bearings_A[ch], svd)
            if diff > 60:
                multi_source_suspect[ch] = (bearings_A[ch], svd, diff)
                print(f"    疑似多干扰源: A={bearings_A[ch]:.1f}° B={svd:.1f}° 差{diff:.0f}°")
        elif mr == "near":
            if try_clear_at(client, second_point, ch, stats):
                print(f"  频道{ch}: near,清除成功")
        elif mr == "no_signal":
            no_signal_at_B.add(ch)
            print(f"  频道{ch}: no_signal")

    # 步骤4: 计算交会点,按交会角从大到小排序
    points = {"A": POINT_A, "B": second_point}
    bearings_all = {"A": bearings_A, "B": bearings_B}

    targets = compute_clear_targets(bearings_all, points)
    print(f"\n--- 交会点(按交会角从大到小排序) ---")
    for ch, inter, d, diff in targets:
        print(f"  频道{ch}: ({inter[0]:.0f},{inter[1]:.0f}) 距原点{d:.0f}m 交会角{diff:.0f}°")

    # 步骤5: 就近清除(按交会角从大到小,但跳过远距离)
    current_pos = second_point
    for ch, inter, d_from_origin, diff in targets:
        if stats["actions"] >= MAX_ACTIONS - 1:
            break
        if client.current_virtual_time >= MAX_VIRTUAL_TIME:
            print(f"  虚拟时间超{MAX_VIRTUAL_TIME}s,停止")
            break
        d_from_current = dist(inter, current_pos)
        # 跳过远距离目标
        if d_from_current > MAX_MOVE_DIST:
            print(f"  频道{ch}: 距当前位置{d_from_current:.0f}m>{MAX_MOVE_DIST}m,跳过")
            continue
        print(f"  频道{ch}: ({inter[0]:.0f},{inter[1]:.0f}) 距当前位置{d_from_current:.0f}m 交会角{diff:.0f}°")
        if try_clear_at(client, inter, ch, stats):
            print(f"    清除成功!")
            current_pos = inter
        else:
            print(f"    清除失败")
            # 对疑似多干扰源,沿A示向度方向再试
            if ch in multi_source_suspect and stats["actions"] < MAX_ACTIONS - 1:
                theta_A, theta_B, _ = multi_source_suspect[ch]
                # 沿A示向度走500米试清除
                alt_point = move_along_bearing(current_pos[0], current_pos[1], theta_A, 300)
                d_alt = dist(alt_point, current_pos)
                if d_alt < MAX_MOVE_DIST and client.current_virtual_time < MAX_VIRTUAL_TIME:
                    print(f"    多干扰源:沿A示向度{theta_A:.0f}°走300米试清除")
                    if try_clear_at(client, alt_point, ch, stats):
                        print(f"      清除成功!")
                        current_pos = alt_point

    # 步骤6: 对B无信号频道,用第三检测点
    bearings_C = {}
    third_point = None
    no_signal_channels = sorted(no_signal_at_B - stats["channels_cleared"])
    if no_signal_channels and stats["actions"] < MAX_ACTIONS - 3 and client.current_virtual_time < MAX_VIRTUAL_TIME:
        print(f"\n--- 第三检测点处理B无信号频道 {no_signal_channels} ---")
        third_point = pick_second_point(avg_bearing + 90, d=THIRD_POINT_DIST)
        print(f"  第三检测点: ({third_point[0]:.0f},{third_point[1]:.0f})")
        for ch in no_signal_channels:
            if stats["actions"] >= MAX_ACTIONS or client.current_virtual_time >= MAX_VIRTUAL_TIME:
                break
            mr, svd = measure_at(client, third_point, ch, stats)
            if mr == "direction":
                bearings_C[ch] = svd
                print(f"  频道{ch}: direction svd={svd}")
            elif mr == "near":
                if try_clear_at(client, third_point, ch, stats):
                    print(f"  频道{ch}: near,清除成功")

        # 三点交会清除(A、C)
        points["C"] = third_point
        bearings_all["C"] = bearings_C
        remaining = sorted(set(no_signal_channels) - stats["channels_cleared"])
        for ch in remaining:
            if ch not in bearings_C:
                print(f"  频道{ch}: B、C均无信号,放弃(无解型)")
                continue
            if ch in bearings_A and ch in bearings_C:
                inter = triangulate(POINT_A, bearings_A[ch], third_point, bearings_C[ch])
                if inter:
                    d = dist(inter, current_pos)
                    if d < MAX_MOVE_DIST and client.current_virtual_time < MAX_VIRTUAL_TIME:
                        print(f"  频道{ch}: A、C交会({inter[0]:.0f},{inter[1]:.0f}) 距当前位置{d:.0f}m")
                        if try_clear_at(client, inter, ch, stats):
                            print(f"    清除成功!")
                            current_pos = inter

    # 步骤7: /exit
    print("\n--- /exit ---")
    resp = client.exit()
    stats["actions"] += 1

    # 汇总
    print("\n" + "=" * 60)
    print("交会定位 v6 演练汇总")
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
    summary_path = os.path.join(script_dir, "rehearsal_triangulate_v6_summary.json")
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
