# -*- coding: utf-8 -*-
"""
B 题问题 3 演练测试:交会定位法 v7(多检测点+自适应清除)

v6→v7 核心改进:
1. 多检测点策略:A、B1、B2、B3四个检测点
   - B1:垂直于平均示向度方向400米
   - B2:沿X轴400米
   - B3:沿Y轴400米
2. 对每个频道,选交会角最大的检测点对
3. 交会角<20°的频道跳过不清除(避免伪交会点)
4. 清除失败的频道,在交会点附近做3步小范围搜索
5. 时间预算:MAX_VIRTUAL_TIME=1400秒(利用v6的余量)
6. 动作预算:MAX_ACTIONS=45
"""
import argparse
import json
import math
import sys
import os

from robot_client import RobotClient

MAX_ACTIONS = 45
MAX_VIRTUAL_TIME = 1400.0
LOG_SIZE_LIMIT = 2 * 1024 * 1024

POINT_A = (0.0, 0.0)
SECOND_POINT_DIST = 400.0
MAX_MOVE_DIST = 800.0
MIN_CUT_ANGLE = 20.0  # 交会角小于此值则跳过


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


def pick_perpendicular_point(bearing_deg, d=SECOND_POINT_DIST):
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


def compute_best_target(channel, bearings_all, points):
    """对单个频道,找交会角最大的检测点对,返回(inter, diff)或None"""
    point_names = list(bearings_all.keys())
    best_inter = None
    best_diff = 0
    for i in range(len(point_names)):
        for j in range(i+1, len(point_names)):
            p1n, p2n = point_names[i], point_names[j]
            if channel not in bearings_all[p1n] or channel not in bearings_all[p2n]:
                continue
            theta1 = bearings_all[p1n][channel]
            theta2 = bearings_all[p2n][channel]
            diff = angle_diff(theta1, theta2)
            if diff < MIN_CUT_ANGLE:
                continue
            inter = triangulate(points[p1n], theta1, points[p2n], theta2)
            if inter and diff > best_diff:
                best_inter = inter
                best_diff = diff
    return (best_inter, best_diff) if best_inter else None


def main():
    parser = argparse.ArgumentParser(description="B 题问题3 演练(交会定位法v7)")
    parser.add_argument("robot_id", help="参赛队号")
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument("--log", default="rehearsal_triangulate_v7_log.jsonl")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(script_dir, args.log)
    if os.path.exists(log_path):
        os.remove(log_path)

    client = RobotClient(args.robot_id, base_url=args.base_url, log_path=log_path)

    print("=" * 60)
    print("B 题问题3 演练(交会定位法 v7)")
    print(f"接口地址: {args.base_url}")
    print(f"robot_id: {args.robot_id}")
    print(f"改进:多检测点A/B1/B2/B3+交会角阈值20°+失败搜索")
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

    # 步骤3: 多检测点(B1=垂直方向, B2=X轴, B3=Y轴)
    to_measure = sorted(set(bearings_A.keys()) - stats["channels_cleared"])
    avg_bearing = sum(bearings_A[ch] for ch in to_measure) / len(to_measure) if to_measure else 0

    # 两个第二检测点(B1=垂直方向, B2=沿X轴)
    points = {"A": POINT_A}
    bearings_all = {"A": bearings_A}
    second_points = {
        "B1": pick_perpendicular_point(avg_bearing),
        "B2": (400.0, 0.0),
    }

    for name, sp in second_points.items():
        if stats["actions"] >= MAX_ACTIONS - 5 or client.current_virtual_time >= MAX_VIRTUAL_TIME:
            break
        print(f"\n--- 检测点{name}=({sp[0]:.0f},{sp[1]:.0f}) ---")
        points[name] = sp
        bearings_tmp = {}
        for ch in to_measure:
            if stats["actions"] >= MAX_ACTIONS or client.current_virtual_time >= MAX_VIRTUAL_TIME:
                break
            mr, svd = measure_at(client, sp, ch, stats)
            if mr == "direction":
                bearings_tmp[ch] = svd
                print(f"  频道{ch}: direction svd={svd}")
            elif mr == "near":
                if try_clear_at(client, sp, ch, stats):
                    print(f"  频道{ch}: near,清除成功")
        bearings_all[name] = bearings_tmp

    # 步骤4: 计算每个频道的最佳交会点(交会角最大的检测点对)
    print(f"\n--- 最佳交会点(交会角>{MIN_CUT_ANGLE}°) ---")
    targets = []
    for ch in stats["channels_with_signal"]:
        if ch in stats["channels_cleared"]:
            continue
        result = compute_best_target(ch, bearings_all, points)
        if result:
            inter, diff = result
            d_origin = dist(inter, (0, 0))
            targets.append((ch, inter, d_origin, diff))
            print(f"  频道{ch}: ({inter[0]:.0f},{inter[1]:.0f}) 距原点{d_origin:.0f}m 交会角{diff:.0f}°")
        else:
            print(f"  频道{ch}: 无交会角>{MIN_CUT_ANGLE}°的检测点对,跳过")

    # 按交会角从大到小排序
    targets.sort(key=lambda t: -t[3])

    # 步骤5: 就近清除
    current_pos = POINT_A
    for ch, inter, d_origin, diff in targets:
        if stats["actions"] >= MAX_ACTIONS - 1:
            break
        if client.current_virtual_time >= MAX_VIRTUAL_TIME:
            print(f"  虚拟时间超{MAX_VIRTUAL_TIME}s,停止")
            break
        d_from_current = dist(inter, current_pos)
        if d_from_current > MAX_MOVE_DIST:
            print(f"  频道{ch}: 距当前位置{d_from_current:.0f}m>{MAX_MOVE_DIST}m,跳过")
            continue
        print(f"  频道{ch}: ({inter[0]:.0f},{inter[1]:.0f}) 距当前位置{d_from_current:.0f}m 交会角{diff:.0f}°")
        if try_clear_at(client, inter, ch, stats):
            print(f"    清除成功!")
            current_pos = inter
        else:
            print(f"    清除失败,在附近搜索...")
            # 小范围网格搜索(3步)
            for offset in [(-20, 0), (20, 0), (0, 20)]:
                if stats["actions"] >= MAX_ACTIONS or client.current_virtual_time >= MAX_VIRTUAL_TIME:
                    break
                alt = (inter[0] + offset[0], inter[1] + offset[1])
                if try_clear_at(client, alt, ch, stats):
                    print(f"      偏移{offset}清除成功!")
                    current_pos = alt
                    break

    # 步骤6: /exit
    print("\n--- /exit ---")
    resp = client.exit()
    stats["actions"] += 1

    # 汇总
    print("\n" + "=" * 60)
    print("交会定位 v7 演练汇总")
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
    summary_path = os.path.join(script_dir, "rehearsal_triangulate_v7_summary.json")
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
