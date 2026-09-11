# -*- coding: utf-8 -*-
"""
B 题问题 3 演练测试:交会定位法 v2(优化版)

改进点:
1. 对 B 点 no_signal 的频道:沿 A 点示向度方向走到半程(500米)再测,获得第二点交会
2. 对交会角过小(两示向度差<10°或>170°)的频道:改用沿示向度逐步逼近法
3. 二次清除失败:在交会点附近做网格搜索(±30米,步长20米)
4. 全程控制动作数,优先清除容易成功的频道
"""
import argparse
import json
import math
import sys
import os

from robot_client import RobotClient

MAX_ACTIONS = 80
LOG_SIZE_LIMIT = 2 * 1024 * 1024

POINT_A = (0.0, 0.0)
POINT_B = (800.0, 0.0)
POINT_C = (0.0, 800.0)
POINT_D = (400.0, 400.0)


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


def grid_search_clear(client, center, channel, stats, radius=40, step=20):
    """在 center 附近做网格搜索清除,返回是否成功"""
    cx, cy = center
    # 螺旋顺序:从中心向外
    offsets = [(0, 0)]
    for r in range(step, radius + 1, step):
        for dx in range(-r, r + 1, step):
            for dy in range(-r, r + 1, step):
                if (dx, dy) not in [(o[0], o[1]) for o in offsets]:
                    offsets.append((dx, dy))

    for dx, dy in offsets:
        if stats["actions"] >= MAX_ACTIONS:
            break
        px, py = cx + dx, cy + dy
        if try_clear_at(client, (px, py), channel, stats):
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description="B 题问题3 演练(交会定位法v2)")
    parser.add_argument("robot_id", help="参赛队号")
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument("--log", default="rehearsal_triangulate_v2_log.jsonl")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(script_dir, args.log)
    if os.path.exists(log_path):
        os.remove(log_path)

    client = RobotClient(args.robot_id, base_url=args.base_url, log_path=log_path)

    print("=" * 60)
    print("B 题问题3 演练(交会定位法 v2)")
    print(f"接口地址: {args.base_url}")
    print(f"robot_id: {args.robot_id}")
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

    # 步骤3: B 点测有信号频道
    to_measure = sorted(set(bearings_A.keys()) - stats["channels_cleared"])
    if to_measure:
        print(f"\n--- B={POINT_B} 测频道 {to_measure} ---")
    bearings_B = {}
    no_signal_at_B = set()
    for ch in to_measure:
        if stats["actions"] >= MAX_ACTIONS:
            break
        mr, svd = measure_at(client, POINT_B, ch, stats)
        if mr == "direction":
            bearings_B[ch] = svd
            print(f"  频道{ch}: direction svd={svd}")
        elif mr == "near":
            if try_clear_at(client, POINT_B, ch, stats):
                print(f"  频道{ch}: near,清除成功")
        elif mr == "no_signal":
            no_signal_at_B.add(ch)
            print(f"  频道{ch}: no_signal(B点超出接收半径)")

    # 步骤4: A、B 交会定位并清除
    print(f"\n--- A、B 交会定位并清除 ---")
    failed = set()
    to_clear = sorted(set(bearings_A.keys()) & set(bearings_B.keys()) - stats["channels_cleared"])
    for ch in to_clear:
        if stats["actions"] >= MAX_ACTIONS:
            break
        theta_A = bearings_A[ch]
        theta_B = bearings_B[ch]
        diff = angle_diff(theta_A, theta_B)
        inter = triangulate(POINT_A, theta_A, POINT_B, theta_B)
        if inter is None:
            print(f"  频道{ch}: A={theta_A}° B={theta_B}° 交会失败(平行)")
            failed.add(ch)
            continue
        ix, iy = inter
        print(f"  频道{ch}: A={theta_A}° B={theta_B}° 差角={diff:.1f}° → ({ix:.0f},{iy:.0f})")
        if try_clear_at(client, (ix, iy), ch, stats):
            print(f"    清除成功!")
        else:
            print(f"    清除失败,留待补救")
            failed.add(ch)

    # 步骤5: 对 B 点 no_signal 的频道,沿 A 示向度走半程再测
    if no_signal_at_B and stats["actions"] < MAX_ACTIONS:
        print(f"\n--- 对 B 点无信号频道 {sorted(no_signal_at_B)} 沿 A 示向度半程补测 ---")
        for ch in sorted(no_signal_at_B):
            if stats["actions"] >= MAX_ACTIONS:
                break
            theta_A = bearings_A[ch]
            # 沿示向度走 500 米
            mid = move_along_bearing(0, 0, theta_A, 500)
            print(f"  频道{ch}: 从 A 沿 {theta_A}° 走500米到 ({mid[0]:.0f},{mid[1]:.0f})")
            mr, svd = measure_at(client, mid, ch, stats)
            if mr == "direction":
                # 用 A 和中点交会
                inter = triangulate(POINT_A, theta_A, mid, svd)
                if inter:
                    ix, iy = inter
                    print(f"    补测 svd={svd}° 交会({ix:.0f},{iy:.0f})")
                    if try_clear_at(client, (ix, iy), ch, stats):
                        print(f"    清除成功!")
                    else:
                        failed.add(ch)
                else:
                    failed.add(ch)
            elif mr == "near":
                if try_clear_at(client, mid, ch, stats):
                    print(f"    near,清除成功!")
            else:
                # 半程仍无信号,可能方向不对,继续走到1000米
                far = move_along_bearing(0, 0, theta_A, 1000)
                mr2, svd2 = measure_at(client, far, ch, stats)
                if mr2 == "direction":
                    inter = triangulate(POINT_A, theta_A, far, svd2)
                    if inter:
                        ix, iy = inter
                        print(f"    远测 svd={svd2}° 交会({ix:.0f},{iy:.0f})")
                        if try_clear_at(client, (ix, iy), ch, stats):
                            print(f"    清除成功!")
                        else:
                            failed.add(ch)
                    else:
                        failed.add(ch)
                elif mr2 == "near":
                    if try_clear_at(client, far, ch, stats):
                        print(f"    near,清除成功!")
                else:
                    failed.add(ch)

    # 步骤6: 对仍失败的频道,用 C 点测并交会
    if failed and stats["actions"] < MAX_ACTIONS:
        print(f"\n--- C={POINT_C} 处理失败频道 {sorted(failed)} ---")
        bearings_C = {}
        for ch in sorted(failed):
            if stats["actions"] >= MAX_ACTIONS:
                break
            mr, svd = measure_at(client, POINT_C, ch, stats)
            if mr == "direction":
                bearings_C[ch] = svd
                print(f"  频道{ch}: direction svd={svd}")
            elif mr == "near":
                if try_clear_at(client, POINT_C, ch, stats):
                    print(f"  频道{ch}: near,清除成功")

        for ch in sorted(failed):
            if ch in stats["channels_cleared"] or ch not in bearings_C:
                continue
            if stats["actions"] >= MAX_ACTIONS:
                break
            # 用 B、C 交会(优先),再 A、C
            inter = None
            if ch in bearings_B:
                inter = triangulate(POINT_B, bearings_B[ch], POINT_C, bearings_C[ch])
            if inter is None and ch in bearings_A:
                inter = triangulate(POINT_A, bearings_A[ch], POINT_C, bearings_C[ch])
            if inter is None:
                continue
            ix, iy = inter
            print(f"  频道{ch}: 二次交会({ix:.0f},{iy:.0f})")
            if try_clear_at(client, (ix, iy), ch, stats):
                print(f"    清除成功!")
            else:
                # 网格搜索
                print(f"    网格搜索...")
                if grid_search_clear(client, (ix, iy), ch, stats):
                    print(f"    网格搜索清除成功!")

    # 步骤7: 仍然失败的,沿 A 示向度逐步逼近
    still_failed = sorted(failed - stats["channels_cleared"])
    if still_failed and stats["actions"] < MAX_ACTIONS:
        print(f"\n--- 沿示向度逐步逼近频道 {still_failed} ---")
        for ch in still_failed:
            if stats["actions"] >= MAX_ACTIONS - 5:
                break
            theta = bearings_A[ch]
            x, y = 0.0, 0.0
            dist = 400
            for step in range(4):
                if stats["actions"] >= MAX_ACTIONS - 1:
                    break
                nx, ny = move_along_bearing(x, y, theta, dist)
                mr, svd = measure_at(client, (nx, ny), ch, stats)
                x, y = nx, ny
                if mr == "direction":
                    theta = svd  # 更新示向度
                    dist = max(dist / 2, 30)
                elif mr == "near":
                    if try_clear_at(client, (x, y), ch, stats):
                        print(f"  频道{ch}: 逼近后清除成功!")
                    break
                else:
                    break

    # 步骤8: /exit
    print("\n--- /exit ---")
    resp = client.exit()
    stats["actions"] += 1

    # 汇总
    print("\n" + "=" * 60)
    print("交会定位 v2 演练汇总")
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
    summary_path = os.path.join(script_dir, "rehearsal_triangulate_v2_summary.json")
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
