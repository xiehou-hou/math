# -*- coding: utf-8 -*-
"""
B 题问题 3 演练测试:交会定位法策略

核心思路:
1. /enter
2. 在 A=(0,0) 扫描频道 1-20,记录有信号频道及示向度
3. 移动到 B=(800,0),对有信号频道测示向度
4. 用 A、B 两点示向度做交会定位,前往交点 /clear
5. 失败的频道,移动到 C=(0,800) 再测,用 B、C 或 A、C 交会重新定位清除
6. 仍然失败的,在 D=(400,400) 测一次,二次定位
7. /exit

交会定位原理:
- 射线1: P1 + s*(cos θ1, sin θ1), s>=0
- 射线2: P2 + t*(cos θ2, sin θ2), t>=0
- 解方程组求交点
"""
import argparse
import json
import math
import sys
import os

from robot_client import RobotClient

MAX_ACTIONS = 80
LOG_SIZE_LIMIT = 2 * 1024 * 1024

# 三个检测点(形成三角形,保证交会角较大)
POINT_A = (0.0, 0.0)
POINT_B = (800.0, 0.0)
POINT_C = (0.0, 800.0)
POINT_D = (400.0, 400.0)  # 备用第四点


def triangulate(p1, theta1_deg, p2, theta2_deg):
    """
    两条射线交点
    射线1: P1 + s*(cos θ1, sin θ1), s>=0
    射线2: P2 + t*(cos θ2, sin θ2), t>=0
    返回 (ix, iy) 或 None(平行或交点在反方向)
    """
    x1, y1 = p1
    x2, y2 = p2
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    d1x, d1y = math.cos(t1), math.sin(t1)
    d2x, d2y = math.cos(t2), math.sin(t2)
    # 方程: s*d1 - t*d2 = P2 - P1
    # D = -d1x*d2y + d2x*d1y
    D = -d1x * d2y + d2x * d1y
    if abs(D) < 1e-9:
        return None  # 平行
    dx = x2 - x1
    dy = y2 - y1
    s = (-dx * d2y + d2x * dy) / D
    if s < -1.0:  # 交点在射线反方向(允许微小负值应对误差)
        return None
    ix = x1 + s * d1x
    iy = y1 + s * d1y
    return (ix, iy)


def measure_at(client, point, channel, stats):
    """在指定点测指定频道,返回 (measure_result, svd_deg 或 None)"""
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
    """在指定点清除指定频道,返回是否成功"""
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


def main():
    parser = argparse.ArgumentParser(description="B 题问题3 演练(交会定位法)")
    parser.add_argument("robot_id", help="参赛队号")
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument("--log", default="rehearsal_triangulate_log.jsonl")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(script_dir, args.log)
    if os.path.exists(log_path):
        os.remove(log_path)

    client = RobotClient(args.robot_id, base_url=args.base_url, log_path=log_path)

    print("=" * 60)
    print("B 题问题3 演练(交会定位法策略)")
    print(f"接口地址: {args.base_url}")
    print(f"robot_id: {args.robot_id}")
    print(f"检测点: A={POINT_A} B={POINT_B} C={POINT_C} D={POINT_D}")
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

    # 步骤2: 在 A=(0,0) 扫描频道 1-20
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
            # 距离过近,直接清除
            if try_clear_at(client, POINT_A, ch, stats):
                print(f"  频道{ch}: near,清除成功")

    print(f"\nA 点扫描完成,有信号频道: {sorted(bearings_A.keys())}")

    if not bearings_A and not stats["channels_cleared"]:
        print("没有检测到信号,退出。")
        client.exit()
        stats["actions"] += 1
        finalize(stats, client, log_path)
        return 0

    # 步骤3: 移动到 B,对有信号(且未清除)频道测示向度
    to_measure_B = sorted(set(bearings_A.keys()) - stats["channels_cleared"])
    if to_measure_B:
        print(f"\n--- B={POINT_B} 测有信号频道 {to_measure_B} ---")
    bearings_B = {}
    for ch in to_measure_B:
        if stats["actions"] >= MAX_ACTIONS:
            break
        mr, svd = measure_at(client, POINT_B, ch, stats)
        if mr == "direction":
            bearings_B[ch] = svd
            print(f"  频道{ch}: direction svd={svd}")
        elif mr == "near":
            if try_clear_at(client, POINT_B, ch, stats):
                print(f"  频道{ch}: near,清除成功")

    # 步骤4: A、B 交会定位并清除
    print(f"\n--- A、B 交会定位并清除 ---")
    inter_points = {}  # 频道 -> 交会点(用于后续二次定位参考)
    failed = set()
    to_clear = sorted(set(bearings_A.keys()) & set(bearings_B.keys()) - stats["channels_cleared"])
    for ch in to_clear:
        if stats["actions"] >= MAX_ACTIONS:
            break
        theta_A = bearings_A[ch]
        theta_B = bearings_B[ch]
        inter = triangulate(POINT_A, theta_A, POINT_B, theta_B)
        if inter is None:
            print(f"  频道{ch}: A示向{theta_A}° B示向{theta_B}° → 交会失败(近似平行)")
            failed.add(ch)
            continue
        ix, iy = inter
        inter_points[ch] = (ix, iy)
        print(f"  频道{ch}: A={theta_A}° B={theta_B}° → 交会点({ix:.0f},{iy:.0f})")
        if try_clear_at(client, (ix, iy), ch, stats):
            print(f"    清除成功!")
        else:
            print(f"    清除失败,留待二次定位")
            failed.add(ch)

    # 步骤5: 对失败的频道,移动到 C 测,用 B、C 或 A、C 交会
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

        # 用 B、C 交会(优先,交会角较大),失败再用 A、C
        still_failed = set()
        for ch in sorted(failed):
            if ch in stats["channels_cleared"] or ch not in bearings_C:
                if ch not in stats["channels_cleared"]:
                    still_failed.add(ch)
                continue
            inter = None
            if ch in bearings_B:
                inter = triangulate(POINT_B, bearings_B[ch], POINT_C, bearings_C[ch])
            if inter is None and ch in bearings_A:
                inter = triangulate(POINT_A, bearings_A[ch], POINT_C, bearings_C[ch])
            if inter is None:
                still_failed.add(ch)
                continue
            ix, iy = inter
            print(f"  频道{ch}: 二次交会({ix:.0f},{iy:.0f})")
            if try_clear_at(client, (ix, iy), ch, stats):
                print(f"    清除成功!")
            else:
                still_failed.add(ch)
        failed = still_failed

    # 步骤6: 仍然失败的,在 D=(400,400) 再测一次,用 C、D 交会
    if failed and stats["actions"] < MAX_ACTIONS - 5:
        print(f"\n--- D={POINT_D} 三次定位失败频道 {sorted(failed)} ---")
        bearings_D = {}
        for ch in sorted(failed):
            if stats["actions"] >= MAX_ACTIONS:
                break
            mr, svd = measure_at(client, POINT_D, ch, stats)
            if mr == "direction":
                bearings_D[ch] = svd
                print(f"  频道{ch}: direction svd={svd}")
            elif mr == "near":
                if try_clear_at(client, POINT_D, ch, stats):
                    print(f"  频道{ch}: near,清除成功")

        for ch in sorted(failed):
            if ch in stats["channels_cleared"] or ch not in bearings_D:
                continue
            if stats["actions"] >= MAX_ACTIONS:
                break
            # 用 C、D 交会
            inter = None
            if ch in bearings_C:
                inter = triangulate(POINT_C, bearings_C[ch], POINT_D, bearings_D[ch])
            if inter is None and ch in bearings_B:
                inter = triangulate(POINT_B, bearings_B[ch], POINT_D, bearings_D[ch])
            if inter is None and ch in bearings_A:
                inter = triangulate(POINT_A, bearings_A[ch], POINT_D, bearings_D[ch])
            if inter is None:
                continue
            ix, iy = inter
            print(f"  频道{ch}: 三次交会({ix:.0f},{iy:.0f})")
            if try_clear_at(client, (ix, iy), ch, stats):
                print(f"    清除成功!")

    # 步骤7: /exit
    print("\n--- /exit ---")
    resp = client.exit()
    stats["actions"] += 1

    # 汇总
    print("\n" + "=" * 60)
    print("交会定位演练汇总")
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
    summary_path = os.path.join(script_dir, "rehearsal_triangulate_summary.json")
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
