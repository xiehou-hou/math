# -*- coding: utf-8 -*-
"""
B 题接口连通测试

步骤:
1. /enter
2. (0,0) /measure 频道1
3. (100,0) /measure 频道1
4. (0,0) /clear 频道1
5. /exit

记录每一步的请求内容、响应内容、虚拟时间、measure_result、svd_deg、clear_result
参赛队号通过命令行参数传入,不写入代码文件

用法:
    python connectivity_test.py <参赛队号>
    python connectivity_test.py <参赛队号> --base-url http://127.0.0.1:2026
"""
import argparse
import json
import sys
import os

from robot_client import RobotClient


def main():
    parser = argparse.ArgumentParser(description="B 题接口连通测试")
    parser.add_argument("robot_id", help="参赛队号(作为 robot_id)")
    parser.add_argument("--base-url", default="http://127.0.0.1:2026", help="模拟器接口地址")
    parser.add_argument("--log", default="connectivity_log.jsonl", help="日志输出文件路径")
    args = parser.parse_args()

    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), args.log)

    # 清空旧日志
    if os.path.exists(log_path):
        os.remove(log_path)

    client = RobotClient(args.robot_id, base_url=args.base_url, log_path=log_path)

    print("=" * 60)
    print("B 题接口连通测试")
    print(f"接口地址: {args.base_url}")
    print(f"robot_id: {args.robot_id}")
    print(f"日志文件: {log_path}")
    print("=" * 60)

    results = {"steps": [], "ok": True, "errors": []}

    # 步骤1: /enter
    print("\n--- 步骤1: /enter ---")
    resp = client.enter()
    step1 = {"action": "/enter", "request": {"arena_id": "default", "robot_id": args.robot_id}, "response": resp}
    results["steps"].append(step1)
    if not resp.get("accepted"):
        results["ok"] = False
        results["errors"].append("/enter 失败: " + json.dumps(resp, ensure_ascii=False))
        print("enter 失败,后续步骤可能无法执行。")
        # 不直接退出,继续尝试后续步骤看接口是否响应
    enter_ok = resp.get("accepted", False)

    # 步骤2: (0,0) /measure 频道1
    print("\n--- 步骤2: (0,0) /measure 频道1 ---")
    resp = client.measure(0, 0, 1)
    step2 = {"action": "/measure", "position": [0, 0], "channel": 1, "response": resp}
    results["steps"].append(step2)
    if not resp.get("accepted"):
        results["ok"] = False
        results["errors"].append("/measure(0,0,1) 未被接受: " + json.dumps(resp, ensure_ascii=False))

    # 步骤3: (100,0) /measure 频道1
    print("\n--- 步骤3: (100,0) /measure 频道1 ---")
    resp = client.measure(100, 0, 1)
    step3 = {"action": "/measure", "position": [100, 0], "channel": 1, "response": resp}
    results["steps"].append(step3)
    if not resp.get("accepted"):
        results["ok"] = False
        results["errors"].append("/measure(100,0,1) 未被接受: " + json.dumps(resp, ensure_ascii=False))

    # 步骤4: (0,0) /clear 频道1
    print("\n--- 步骤4: (0,0) /clear 频道1 ---")
    resp = client.clear(0, 0, 1)
    step4 = {"action": "/clear", "position": [0, 0], "channel": 1, "response": resp}
    results["steps"].append(step4)
    if not resp.get("accepted"):
        results["ok"] = False
        results["errors"].append("/clear(0,0,1) 未被接受: " + json.dumps(resp, ensure_ascii=False))

    # 步骤5: /exit
    print("\n--- 步骤5: /exit ---")
    resp = client.exit()
    step5 = {"action": "/exit", "response": resp}
    results["steps"].append(step5)
    if not resp.get("accepted"):
        results["ok"] = False
        results["errors"].append("/exit 未被接受: " + json.dumps(resp, ensure_ascii=False))

    # 汇总
    print("\n" + "=" * 60)
    print("连通测试汇总")
    print("=" * 60)

    # 提取关键字段
    for i, step in enumerate(results["steps"], 1):
        r = step.get("response", {})
        mr = r.get("measure_result", "-")
        svd = r.get("svd_deg", "-")
        cr = r.get("clear_result", "-")
        er = r.get("exit_reason", "-")
        vt = r.get("virtual_time_s", "-")
        acc = r.get("accepted", False)
        print(f"步骤{i} {step['action']}: accepted={acc} vt={vt} measure_result={mr} svd_deg={svd} clear_result={cr} exit_reason={er}")

    print(f"\n整体结果: {'通过' if results['ok'] else '有错误'}")
    if results["errors"]:
        for err in results["errors"]:
            print(f"  错误: {err}")

    # 检查日志大小
    log_size = os.path.getsize(log_path) if os.path.exists(log_path) else 0
    print(f"\n日志文件大小: {log_size} 字节 ({log_size / 1024:.1f} KB)")
    if log_size > 2 * 1024 * 1024:
        print("警告: 日志文件超过 2MB!")

    # 保存汇总
    summary_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "connectivity_summary.json")
    with open(summary_path, "w", encoding="utf-8") as fout:
        json.dump(results, fout, ensure_ascii=False, indent=2)
    print(f"汇总已保存: {summary_path}")

    return 0 if results["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
