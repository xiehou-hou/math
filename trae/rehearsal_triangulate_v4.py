# -*- coding: utf-8 -*-
"""
B 题问题 3 演练测试:交会定位法 v4

v3→v4 核心改进:
1. 网格搜索预算收紧到3次,且仅在交会角>30°时才做(交会角小则定位本身不可靠)
2. "无解型"频道尽早放弃:B无信号且补测示向度与A差<15°且C也无信号,直接放弃
3. 频道优先级排序:先清除交会角大的(容易成功),后处理交会角小的
4. 网格搜索位置偏移:不在原交会点(已失败),而在交会点沿示向度方向偏移20米处开始
5. B无信号频道直接用第三点,不再补测(补测浪费2个动作且常失败)
6. 全局动作上限55,每频道预算5
"""
import argparse
import json
import math
import sys
import os

from robot_client import RobotClient

MAX_ACTIONS = 55
BUDGET_PER_CHANNEL = 5
LOG_SIZE_LIMIT = 2 * 1024 * 1024

POINT_A = (0.0, 0.0)
SECOND_POINT_DIST = 500.0
GRID_RADIUS = 40
GRID_STEP = 20
GRID_BUDGET = 3  # 网格搜索次数(收紧)
APPROACH_MAX_STEPS = 2


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


def pick_second_point(bearing_deg, dist=SECOND_POINT_DIST):
    perp1 = (bearing_deg + 90) % 360
    p1 = move_along_bearing(0, 0, perp1, dist)
    p2 = move_along_bearing(0, 0, (bearing_deg - 90) % 360, dist)
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


def grid_search_clear(client, center, channel, stats, bearing_hint=None, budget=GRID_BUDGET):
    """网格搜索,从交会点沿示向度偏移开始
    bearing_hint: 交会点处示向度方向(用于偏移起点)"""
    cx, cy = center
    # 起点偏移:沿示向度方向20米
    if bearing_hint is not None:
        ox, oy = move_along_bearing(0, 0, bearing_hint, GRID_STEP)
    else:
        ox, oy = 0, 0

    offsets = [(int(ox), int(oy))]
    for r in range(GRID_STEP, GRID_RADIUS + 1, GRID_STEP):
        for dx in range(-r, r + 1, GRID_STEP):
            for dy in range(-r, r + 1, GRID_STEP):
                pos = (int(ox) + dx, int(oy) + dy)
                if pos not in offsets:
                    offsets.append(pos)

    count = 0
    for dx, dy in offsets:
        if count >= budget or stats["actions"] >= MAX_ACTIONS:
            break
        if try_clear_at(client, (cx + dx, cy + dy), channel, stats):
            return True
        count += 1
    return False


def clear_channel(client, ch, bearings, points, stats, budget):
    """清除单个频道,按交会角从大到小尝试检测点对"""
    used = 0
    point_names = list(bearings.keys())

    # 构建所有检测点对,按交会角从大到小排序(交会角大=定位准)
    pairs = []
    for i in range(len(point_names)):
        for j in range(i + 1, len(point_names)):
            p1n, p2n = point_names[i], point_names[j]
            diff = angle_diff(bearings[p1n], bearings[p2n])
            pairs.append((diff, p1n, p2n))
    pairs.sort(reverse=True)  # 交会角大的优先

    for diff, p1n, p2n in pairs:
        if used >= budget or stats["actions"] >= MAX_ACTIONS:
            break
        theta1, theta2 = bearings[p1n], bearings[p2n]
        p1, p2 = points[p1n], points[p2n]

        # 差角<15°:中点策略
        if diff < 15:
            mid = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
            print(f"    {p1n}={theta1:.1f}° {p2n}={theta2:.1f}° 差角={diff:.0f}°<15° → 中点({mid[0]:.0f},{mid[1]:.0f})")
            if try_clear_at(client, mid, ch, stats):
                print(f"    中点清除成功!")
                return True
            used += 1
            continue

        inter = triangulate(p1, theta1, p2, theta2)
        if inter is None:
            continue
        ix, iy = inter
        print(f"    {p1n}={theta1:.1f}° {p2n}={theta2:.1f}° 差角={diff:.0f}° → ({ix:.0f},{iy:.0f})")
        if try_clear_at(client, (ix, iy), ch, stats):
            print(f"    清除成功!")
            return True
        used += 1

        # 网格搜索(仅交会角>30°时,且预算足够)
        if diff > 30 and budget - used >= 3:
            print(f"    网格搜索(预算3)")
            if grid_search_clear(client, (ix, iy), ch, stats, bearing_hint=theta1, budget=3):
                print(f"    网格搜索清除成功!")
                return True
            used += 3

    return ch in stats["channels_cleared"]


def main():
    parser = argparse.ArgumentParser(description="B 题问题3 演练(交会定位法v4)")
    parser.add_argument("robot_id", help="参赛队号")
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument("--log", default="rehearsal_triangulate_v4_log.jsonl")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(script_dir, args.log)
    if os.path.exists(log_path):
        os.remove(log_path)

    client = RobotClient(args.robot_id, base_url=args.base_url, log_path=log_path)

    print("=" * 60)
    print("B 题问题3 演练(交会定位法 v4)")
    print(f"接口地址: {args.base_url}")
    print(f"robot_id: {args.robot_id}")
    print(f"改进:优先级排序+网格收紧+无解放弃+B无信号直接第三点")
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

    # 步骤4: 按交会角排序,优先清除容易成功的频道
    # 计算每个频道的最大交会角(用于排序)
    channel_priority = []
    for ch in to_measure:
        if ch in stats["channels_cleared"]:
            continue
        max_diff = 0
        if ch in bearings_A and ch in bearings_B:
            max_diff = angle_diff(bearings_A[ch], bearings_B[ch])
        channel_priority.append((max_diff, ch))
    channel_priority.sort(reverse=True)  # 交会角大的优先

    print(f"\n--- 交会定位并清除(按交会角排序) ---")
    print(f"  优先级: {[(ch, f'{d:.0f}°') for d, ch in channel_priority]}")
    points = {"A": POINT_A, "B": second_point}
    failed = set()

    for _, ch in channel_priority:
        if stats["actions"] >= MAX_ACTIONS - 2:
            break
        print(f"  频道{ch}:")
        bearings = {}
        if ch in bearings_A:
            bearings["A"] = bearings_A[ch]
        if ch in bearings_B:
            bearings["B"] = bearings_B[ch]

        if len(bearings) >= 2:
            ok = clear_channel(client, ch, bearings, points, stats, budget=BUDGET_PER_CHANNEL)
            if not ok:
                failed.add(ch)
                print(f"    频道{ch} 未清除")
        else:
            # B无信号,留待第三点处理
            failed.add(ch)
            print(f"    B无信号,留待第三点")

    # 步骤5: 对失败频道,用第三检测点
    if failed and stats["actions"] < MAX_ACTIONS - 3:
        print(f"\n--- 第三检测点处理失败频道 {sorted(failed)} ---")
        third_point = pick_second_point(avg_bearing + 90)
        print(f"  第三检测点: ({third_point[0]:.0f},{third_point[1]:.0f})")
        bearings_C = {}
        for ch in sorted(failed):
            if stats["actions"] >= MAX_ACTIONS:
                break
            mr, svd = measure_at(client, third_point, ch, stats)
            if mr == "direction":
                bearings_C[ch] = svd
                print(f"  频道{ch}: direction svd={svd}")
            elif mr == "near":
                if try_clear_at(client, third_point, ch, stats):
                    print(f"  频道{ch}: near,清除成功")

        points["C"] = third_point
        for ch in sorted(failed):
            if ch in stats["channels_cleared"]:
                continue
            if stats["actions"] >= MAX_ACTIONS - 2:
                break
            # 无解型判断: B无信号 + C无信号 → 放弃
            if ch not in bearings_B and ch not in bearings_C:
                print(f"  频道{ch}: B、C均无信号,放弃(无解型)")
                continue
            print(f"  频道{ch}: 三点交会")
            bearings = {}
            if ch in bearings_A:
                bearings["A"] = bearings_A[ch]
            if ch in bearings_B:
                bearings["B"] = bearings_B[ch]
            if ch in bearings_C:
                bearings["C"] = bearings_C[ch]
            clear_channel(client, ch, bearings, points, stats, budget=BUDGET_PER_CHANNEL)

    # 步骤6: /exit
    print("\n--- /exit ---")
    resp = client.exit()
    stats["actions"] += 1

    # 汇总
    print("\n" + "=" * 60)
    print("交会定位 v4 演练汇总")
    print("=" * 60)
    print(f"总动作数: {stats['actions']}")
    print(f"有信号频道: {sorted(stats['channels_with_signal'])}")
    print(f"清除干扰源数: {stats['cleared']}")
    print(f"清除的频道: {sorted(stats['channels_cleared'])}")
    print(f"虚拟时间: {client.current_virtual_time:.2f} 秒")
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
    summary_path = os.path.join(script_dir, "rehearsal_triangulate_v4_summary.json")
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
