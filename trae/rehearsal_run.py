# -*- coding: utf-8 -*-
"""
B 题问题 3 演练测试:简单扫描策略

策略(只追求跑通流程,不追求最优):
1. /enter 进入
2. 从 (0,0) 出发,初始频道1
3. 在当前点遍历频道 1-20,记录有信号的频道及示向度
4. 对有 direction 的频道,沿示向度方向移动一段距离后再测
5. 距离足够近(measure_result=near)时尝试 /clear
6. /exit 退出

控制:
- 每个频道最多追踪 MAX_STEPS_PER_CHANNEL 步
- 总动作数上限 MAX_ACTIONS
- 日志文件大小检查
- 使用 remaining_real_duration_s 控制运行时间

用法:
    python rehearsal_run.py <参赛队号>
    python rehearsal_run.py <参赛队号> --base-url http://127.0.0.1:2026
"""
import argparse
import json
import math
import sys
import os

from robot_client import RobotClient

# 每个频道最多追踪步数
MAX_STEPS_PER_CHANNEL = 6
# 总动作数上限(防止超时和日志过大)
MAX_ACTIONS = 200
# 初始移动距离(米)
INITIAL_MOVE_M = 500
# 最小移动距离(米)
MIN_MOVE_M = 50
# 日志大小上限(字节)
LOG_SIZE_LIMIT = 2 * 1024 * 1024  # 2MB


def move_along_bearing(x: float, y: float, bearing_deg: float, dist_m: float):
    """沿示向度方向移动 dist_m 米,返回新坐标"""
    rad = math.radians(bearing_deg)
    nx = x + dist_m * math.cos(rad)
    ny = y + dist_m * math.sin(rad)
    return nx, ny


def main():
    parser = argparse.ArgumentParser(description="B 题问题3 演练测试(简单扫描)")
    parser.add_argument("robot_id", help="参赛队号")
    parser.add_argument("--base-url", default="http://127.0.0.1:2026", help="模拟器接口地址")
    parser.add_argument("--log", default="rehearsal_log.jsonl", help="日志文件路径")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(script_dir, args.log)
    if os.path.exists(log_path):
        os.remove(log_path)

    client = RobotClient(args.robot_id, base_url=args.base_url, log_path=log_path)

    print("=" * 60)
    print("B 题问题3 演练测试(简单扫描策略)")
    print(f"接口地址: {args.base_url}")
    print(f"robot_id: {args.robot_id}")
    print(f"日志文件: {log_path}")
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
        print("enter 失败,无法继续。")
        finalize(stats, client, log_path)
        return 1

    remaining = client.remaining_real_duration_s or 1200
    print(f"本局可用现实时间: {remaining} 秒")

    # 步骤2: 在 (0,0) 遍历频道 1-20
    print("\n--- (0,0) 扫描频道 1-20 ---")
    x, y = 0.0, 0.0
    # 频道 -> 示向度
    bearing_map = {}

    for ch in range(1, 21):
        if stats["actions"] >= MAX_ACTIONS:
            print(f"达到总动作数上限 {MAX_ACTIONS},停止扫描。")
            break
        resp = client.measure(x, y, ch)
        stats["actions"] += 1
        if resp.get("accepted"):
            mr = resp.get("measure_result")
            if mr == "direction":
                svd = resp.get("svd_deg")
                bearing_map[ch] = svd
                stats["channels_with_signal"].add(ch)
                print(f"  频道{ch}: 有信号 svd_deg={svd}")
            elif mr == "near":
                # 距离过近,直接尝试清除
                print(f"  频道{ch}: near,直接尝试清除")
                stats["channels_with_signal"].add(ch)
                cresp = client.clear(x, y, ch)
                stats["actions"] += 1
                if cresp.get("accepted") and cresp.get("clear_result") == "success":
                    stats["cleared"] += 1
                    stats["channels_cleared"].add(ch)
                    print(f"  频道{ch}: 清除成功!")
                else:
                    print(f"  频道{ch}: 清除未成功")
            else:
                print(f"  频道{ch}: no_signal")
        else:
            stats["errors"].append(f"/measure(0,0,{ch}) 未被接受")

    print(f"\n扫描完成: {len(bearing_map)} 个频道有 direction 信号")
    print(f"有信号频道: {sorted(stats['channels_with_signal'])}")

    # 步骤3: 对有 direction 的频道,沿示向度方向追踪
    for ch, bearing in list(bearing_map.items()):
        if ch in stats["channels_cleared"]:
            continue
        if stats["actions"] >= MAX_ACTIONS:
            print(f"达到总动作数上限,停止追踪。")
            break

        print(f"\n--- 追踪频道 {ch},示向度 {bearing} ---")
        cx, cy = x, y
        move_dist = INITIAL_MOVE_M

        for step in range(MAX_STEPS_PER_CHANNEL):
            if stats["actions"] >= MAX_ACTIONS:
                break
            # 沿示向度方向移动
            nx, ny = move_along_bearing(cx, cy, bearing, move_dist)
            print(f"  步骤{step+1}: 从({cx:.0f},{cy:.0f})移动{move_dist:.0f}米到({nx:.0f},{ny:.0f})")

            resp = client.measure(nx, ny, ch)
            stats["actions"] += 1
            cx, cy = nx, ny

            if not resp.get("accepted"):
                stats["errors"].append(f"/measure 追踪频道{ch} 未被接受")
                break

            mr = resp.get("measure_result")
            if mr == "direction":
                bearing = resp.get("svd_deg")  # 更新示向度
                print(f"  仍有信号,新示向度={bearing}")
                # 距离减半,继续接近
                move_dist = max(move_dist / 2, MIN_MOVE_M)
            elif mr == "near":
                print(f"  距离过近,尝试清除")
                cresp = client.clear(cx, cy, ch)
                stats["actions"] += 1
                if cresp.get("accepted") and cresp.get("clear_result") == "success":
                    stats["cleared"] += 1
                    stats["channels_cleared"].add(ch)
                    print(f"  频道{ch}: 清除成功!")
                else:
                    print(f"  频道{ch}: 清除未成功")
                break
            else:
                print(f"  no_signal,停止追踪频道{ch}")
                break

    # 步骤4: /exit
    print("\n--- /exit ---")
    resp = client.exit()
    stats["actions"] += 1

    # 汇总
    print("\n" + "=" * 60)
    print("演练测试汇总")
    print("=" * 60)
    print(f"总动作数: {stats['actions']}")
    print(f"发现信号频道数: {len(stats['channels_with_signal'])}")
    print(f"清除干扰源数: {stats['cleared']}")
    print(f"清除的频道: {sorted(stats['channels_cleared'])}")
    print(f"虚拟时间: {client.current_virtual_time:.2f} 秒")
    if stats["errors"]:
        print(f"错误数: {len(stats['errors'])}")
        for err in stats["errors"][:10]:
            print(f"  {err}")

    log_size = os.path.getsize(log_path) if os.path.exists(log_path) else 0
    print(f"\n日志文件大小: {log_size} 字节 ({log_size / 1024:.1f} KB)")
    if log_size > LOG_SIZE_LIMIT:
        print(f"警告: 日志文件超过 2MB!")

    finalize(stats, client, log_path)
    return 0


def finalize(stats, client, log_path):
    """保存汇总"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    summary_path = os.path.join(script_dir, "rehearsal_summary.json")
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
