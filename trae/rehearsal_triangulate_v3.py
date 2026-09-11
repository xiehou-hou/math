# -*- coding: utf-8 -*-
"""
B 题问题 3 演练测试:交会定位法 v3

v2→v3 核心改进:
1. 动态选第二检测点:根据A点示向度,在垂直于示向度方向选第二检测点
   - 使交会角接近90°,提升定位精度
   - 距离500米(确保在接收半径内且移动耗时合理)
2. 补测中点策略:补测点示向度与原示向度差<15°时
   - 说明机器狗在两点连线上,干扰源在两点之间
   - 直接用两点中点作为清除位置,而非交会(平行射线无交点)
3. 时间预算管理:每频道分配6个动作预算
   - 全局动作上限60,留足清除余量
4. 网格搜索修正:交会点清除失败时正确触发
   - 螺旋搜索±40米,步长20米,最多9次
5. 逐步逼近限3步:避免示向度跳变死循环
"""
import argparse
import json
import math
import sys
import os

from robot_client import RobotClient

MAX_ACTIONS = 60  # 全局动作上限
BUDGET_PER_CHANNEL = 6  # 每频道动作预算
LOG_SIZE_LIMIT = 2 * 1024 * 1024

POINT_A = (0.0, 0.0)
SECOND_POINT_DIST = 500.0  # 第二检测点距离A点的距离
GRID_RADIUS = 40  # 网格搜索半径
GRID_STEP = 20  # 网格搜索步长
APPROACH_MAX_STEPS = 3  # 逐步逼近最大步数


def triangulate(p1, theta1_deg, p2, theta2_deg):
    """两条射线交点,返回 (ix, iy) 或 None"""
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
    ix = x1 + s * d1x
    iy = y1 + s * d1y
    return (ix, iy)


def angle_diff(a, b):
    """两角度差(度),归一化到 [0, 180]"""
    d = abs(a - b) % 360
    if d > 180:
        d = 360 - d
    return d


def move_along_bearing(x, y, bearing_deg, dist_m):
    rad = math.radians(bearing_deg)
    return x + dist_m * math.cos(rad), y + dist_m * math.sin(rad)


def pick_second_point(bearing_deg, dist=SECOND_POINT_DIST):
    """
    根据A点示向度,选择第二检测点
    策略:在垂直于示向度方向偏移,使交会角接近90°
    垂直方向: 示向度+90° 或 -90°,选距离原点更近的
    """
    # 候选: 示向度+90° 和 -90°
    perp1 = (bearing_deg + 90) % 360
    perp2 = (bearing_deg - 90) % 360
    p1 = move_along_bearing(0, 0, perp1, dist)
    p2 = move_along_bearing(0, 0, perp2, dist)
    # 选距离原点更近的(减少移动耗时),其实距离一样,选x正方向的
    return p1 if p1[0] >= p2[0] else p2


def measure_at(client, point, channel, stats):
    resp = client.measure(point[0], point[1], channel)
    stats["actions"] += 1
    if resp.get("accepted"):
        mr = resp.get("measure_result")
        if mr == "direction":
            return mr, resp.get("svd_deg")
        return mr, None
    else:
        stats["errors"].append(f"measure({point},{channel}) 未被接受")
    return None, None


def try_clear_at(client, point, channel, stats):
    resp = client.clear(point[0], point[1], channel)
    stats["actions"] += 1
    if resp.get("accepted"):
        cr = resp.get("clear_result")
        if cr == "success":
            stats["cleared"] += 1
            stats["channels_cleared"].add(channel)
            return True
        return False
    else:
        stats["errors"].append(f"clear({point},{channel}) 未被接受")
    return False


def grid_search_clear(client, center, channel, stats, budget=5):
    """在 center 附近做网格搜索清除,返回是否成功
    螺旋顺序:从中心向外,最多 budget 次"""
    cx, cy = center
    offsets = [(0, 0)]
    for r in range(GRID_STEP, GRID_RADIUS + 1, GRID_STEP):
        for dx in range(-r, r + 1, GRID_STEP):
            for dy in range(-r, r + 1, GRID_STEP):
                if (dx, dy) not in [(o[0], o[1]) for o in offsets]:
                    offsets.append((dx, dy))

    count = 0
    for dx, dy in offsets:
        if count >= budget or stats["actions"] >= MAX_ACTIONS:
            break
        px, py = cx + dx, cy + dy
        if try_clear_at(client, (px, py), channel, stats):
            return True
        count += 1
    return False


def clear_channel(client, ch, bearings, points, stats, budget):
    """
    清除单个频道(交会+网格搜索+中点策略)
    bearings: {point_name: svd_deg} 已测得的示向度
    points: {point_name: (x,y)} 检测点位置
    budget: 剩余动作预算
    返回是否成功
    """
    used = 0

    # 尝试所有检测点对的交会
    point_names = list(bearings.keys())
    for i in range(len(point_names)):
        for j in range(i + 1, len(point_names)):
            if used >= budget or stats["actions"] >= MAX_ACTIONS:
                return ch in stats["channels_cleared"]
            p1_name, p2_name = point_names[i], point_names[j]
            theta1 = bearings[p1_name]
            theta2 = bearings[p2_name]
            p1 = points[p1_name]
            p2 = points[p2_name]
            diff = angle_diff(theta1, theta2)

            # 判断是否接近平行(差角<15°)
            if diff < 15:
                # 补测中点策略:示向度接近,说明干扰源在两点连线上
                mid = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
                print(f"    差角{diff:.1f}°<15°,用中点({mid[0]:.0f},{mid[1]:.0f})清除")
                if try_clear_at(client, mid, ch, stats):
                    print(f"    中点清除成功!")
                    return True
                used += 1
                continue

            inter = triangulate(p1, theta1, p2, theta2)
            if inter is None:
                continue
            ix, iy = inter
            print(f"    {p1_name}={theta1}° {p2_name}={theta2}° 差角={diff:.0f}° → ({ix:.0f},{iy:.0f})")
            if try_clear_at(client, (ix, iy), ch, stats):
                print(f"    清除成功!")
                return True
            used += 1

            # 交会点清除失败,网格搜索
            print(f"    网格搜索(预算{min(5, budget-used)})")
            if grid_search_clear(client, (ix, iy), ch, stats, budget=min(5, budget - used)):
                print(f"    网格搜索清除成功!")
                return True

    return ch in stats["channels_cleared"]


def main():
    parser = argparse.ArgumentParser(description="B 题问题3 演练(交会定位法v3)")
    parser.add_argument("robot_id", help="参赛队号")
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument("--log", default="rehearsal_triangulate_v3_log.jsonl")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(script_dir, args.log)
    if os.path.exists(log_path):
        os.remove(log_path)

    client = RobotClient(args.robot_id, base_url=args.base_url, log_path=log_path)

    print("=" * 60)
    print("B 题问题3 演练(交会定位法 v3)")
    print(f"接口地址: {args.base_url}")
    print(f"robot_id: {args.robot_id}")
    print(f"改进:动态选点+中点策略+网格搜索+时间预算")
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

    # 步骤3: 对每个有信号频道,动态选第二检测点并测示向度
    # 为了减少移动,按A示向度方向分组,同方向的一次移动
    # 简化处理:对所有频道用同一个第二检测点(垂直于平均示向度)
    to_measure = sorted(set(bearings_A.keys()) - stats["channels_cleared"])
    if to_measure:
        # 计算平均示向度
        avg_bearing = sum(bearings_A[ch] for ch in to_measure) / len(to_measure)
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
    else:
        bearings_B = {}
        no_signal_at_B = set()
        second_point = POINT_A

    # 步骤4: 交会定位并清除(每频道分配预算)
    print(f"\n--- 交会定位并清除 ---")
    points = {"A": POINT_A, "B": second_point}
    failed = set()
    for ch in to_measure:
        if ch in stats["channels_cleared"]:
            continue
        if stats["actions"] >= MAX_ACTIONS - 2:
            break
        print(f"  频道{ch}:")
        bearings = {}
        if ch in bearings_A:
            bearings["A"] = bearings_A[ch]
        if ch in bearings_B:
            bearings["B"] = bearings_B[ch]

        if len(bearings) < 2:
            # 只有一个检测点有信号,需要补测
            if ch in no_signal_at_B:
                # 沿A示向度走半程补测
                theta_A = bearings_A[ch]
                mid_point = move_along_bearing(0, 0, theta_A, 500)
                print(f"    B无信号,沿A示向度{theta_A}°走500米到({mid_point[0]:.0f},{mid_point[1]:.0f})")
                if stats["actions"] < MAX_ACTIONS:
                    mr, svd = measure_at(client, mid_point, ch, stats)
                    if mr == "direction":
                        bearings["M"] = svd
                        points["M"] = mid_point
                        print(f"    补测 svd={svd}°")
                    elif mr == "near":
                        if try_clear_at(client, mid_point, ch, stats):
                            print(f"    near,清除成功!")
                            continue
                    elif mr == "no_signal":
                        # 半程无信号,继续走到1000米
                        far_point = move_along_bearing(0, 0, theta_A, 1000)
                        print(f"    半程无信号,走1000米到({far_point[0]:.0f},{far_point[1]:.0f})")
                        if stats["actions"] < MAX_ACTIONS:
                            mr2, svd2 = measure_at(client, far_point, ch, stats)
                            if mr2 == "direction":
                                bearings["F"] = svd2
                                points["F"] = far_point
                            elif mr2 == "near":
                                if try_clear_at(client, far_point, ch, stats):
                                    print(f"    near,清除成功!")
                                    continue

        if len(bearings) < 2:
            # 仍然只有一个点,用逐步逼近
            if ch in bearings_A:
                theta = bearings_A[ch]
                x, y = 0.0, 0.0
                dist = 400
                for step in range(APPROACH_MAX_STEPS):
                    if stats["actions"] >= MAX_ACTIONS - 1:
                        break
                    nx, ny = move_along_bearing(x, y, theta, dist)
                    mr, svd = measure_at(client, (nx, ny), ch, stats)
                    x, y = nx, ny
                    if mr == "direction":
                        theta = svd
                        dist = max(dist / 2, 30)
                    elif mr == "near":
                        if try_clear_at(client, (x, y), ch, stats):
                            print(f"    逼近清除成功!")
                        break
                    else:
                        break
            continue

        # 交会定位并清除
        ok = clear_channel(client, ch, bearings, points, stats, budget=BUDGET_PER_CHANNEL)
        if not ok:
            failed.add(ch)
            print(f"    频道{ch} 未清除")

    # 步骤5: 对失败频道,用第三个检测点(与A、B不同方向)
    if failed and stats["actions"] < MAX_ACTIONS - 5:
        print(f"\n--- 第三检测点处理失败频道 {sorted(failed)} ---")
        # 选第三个检测点:与A-B连线垂直方向
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
    print("交会定位 v3 演练汇总")
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
    summary_path = os.path.join(script_dir, "rehearsal_triangulate_v3_summary.json")
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
