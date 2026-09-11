# -*- coding: utf-8 -*-
"""
B 题 机器狗最小 HTTP+JSON 客户端

只实现 4 个接口: /enter, /measure, /clear, /exit
- 串行发送,收到完整响应后再发下一个
- 同时检查 HTTP 状态和 accepted 字段
- 记录每一步的请求/响应/虚拟时间/measure_result/svd_deg/clear_result
- 参赛队号通过命令行参数传入,不写入代码文件
"""
import json
import time
import uuid
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError

# 模拟器默认接口地址
BASE_URL = "http://127.0.0.1:2026"
# 超时秒数
TIMEOUT_S = 10


class RobotClient:
    """机器狗客户端,封装 4 个接口的串行调用与日志记录"""

    def __init__(self, robot_id: str, base_url: str = BASE_URL, log_path: str = None):
        self.robot_id = robot_id
        self.base_url = base_url.rstrip("/")
        self.log_path = log_path
        # 当前虚拟时刻,取最近一次 accepted=true 的响应
        self.current_virtual_time = 0.0
        # 剩余现实时间(秒),由 /enter 返回
        self.remaining_real_duration_s = None
        # 请求计数,用于生成 request_id
        self._seq = 0
        # 日志条目列表
        self._log_entries = []
        # 当前测向机频道,由 /measure 推断更新
        self.current_channel = 1

    def _new_request_id(self, prefix: str) -> str:
        """生成新的 request_id: 前缀-序号-短uuid"""
        self._seq += 1
        short = uuid.uuid4().hex[:8]
        return f"{prefix}-{self._seq:04d}-{short}"

    def _append_log(self, entry: dict) -> None:
        self._log_entries.append(entry)
        if self.log_path:
            with open(self.log_path, "a", encoding="utf-8") as fout:
                fout.write(json.dumps(entry, ensure_ascii=False) + "\n")

    def _post(self, path: str, payload: dict, request_id_hint: str = None) -> dict:
        """
        发送 POST 请求并返回解析后的 JSON 响应。
        - 同时检查 HTTP 状态和 accepted 字段
        - 连接失败时返回 {"accepted": False, "error": "..."} 形式
        """
        url = self.base_url + path
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        req = Request(url, data=body, headers=headers, method="POST")

        http_status = None
        try:
            with urlopen(req, timeout=TIMEOUT_S) as resp:
                http_status = resp.getcode()
                raw = resp.read().decode("utf-8")
        except HTTPError as e:
            http_status = e.code
            raw = e.read().decode("utf-8", errors="replace")
        except (URLError, TimeoutError, ConnectionError, OSError) as e:
            # 连接失败,没有 JSON 体
            entry = {
                "step": path,
                "request": payload,
                "http_status": None,
                "error": f"连接失败: {type(e).__name__}: {e}",
                "accepted": False,
            }
            self._append_log(entry)
            print(f"[{path}] 连接失败: {e}")
            return {"accepted": False, "error": str(e), "http_status": None}

        # 尝试解析 JSON
        try:
            resp_json = json.loads(raw)
        except json.JSONDecodeError:
            entry = {
                "step": path,
                "request": payload,
                "http_status": http_status,
                "raw_body": raw[:500],
                "error": "JSON 解析失败",
                "accepted": False,
            }
            self._append_log(entry)
            print(f"[{path}] JSON 解析失败: {raw[:200]}")
            return {"accepted": False, "error": "JSON 解析失败", "http_status": http_status}

        accepted = resp_json.get("accepted", False)
        # 更新虚拟时间(仅 accepted=true 时)
        if accepted and "virtual_time_s" in resp_json:
            self.current_virtual_time = float(resp_json["virtual_time_s"])

        entry = {
            "step": path,
            "request": payload,
            "http_status": http_status,
            "accepted": accepted,
            "response": resp_json,
            "virtual_time_s": resp_json.get("virtual_time_s"),
        }
        self._append_log(entry)
        print(f"[{path}] HTTP={http_status} accepted={accepted} vt={resp_json.get('virtual_time_s')}")
        return resp_json

    def enter(self) -> dict:
        """POST /enter: 进入目标区域"""
        rid = self._new_request_id("enter")
        payload = {
            "arena_id": "default",
            "robot_id": self.robot_id,
            "request_id": rid,
        }
        resp = self._post("/enter", payload)
        if resp.get("accepted"):
            self.remaining_real_duration_s = resp.get("remaining_real_duration_s")
            print(f"  本局可用现实时间: {self.remaining_real_duration_s} 秒")
        return resp

    def measure(self, x: float, y: float, channel: int) -> dict:
        """POST /measure: 移动到 (x,y) 并检测指定频道"""
        rid = self._new_request_id("measure")
        payload = {
            "arena_id": "default",
            "robot_id": self.robot_id,
            "request_id": rid,
            "position": {"x": x, "y": y},
            "channel": channel,
        }
        resp = self._post("/measure", payload)
        if resp.get("accepted"):
            mr = resp.get("measure_result")
            svd = resp.get("svd_deg")
            if mr == "direction":
                print(f"  measure_result=direction svd_deg={svd}")
            elif mr == "near":
                print("  measure_result=near (距离过近,无示向度)")
            else:
                print(f"  measure_result={mr}")
            # 更新当前频道
            self.current_channel = channel
        return resp

    def clear(self, x: float, y: float, channel: int) -> dict:
        """POST /clear: 移动到 (x,y) 并尝试清除指定频道干扰源"""
        rid = self._new_request_id("clear")
        payload = {
            "arena_id": "default",
            "robot_id": self.robot_id,
            "request_id": rid,
            "position": {"x": x, "y": y},
            "channel": channel,
        }
        resp = self._post("/clear", payload)
        if resp.get("accepted"):
            cr = resp.get("clear_result")
            print(f"  clear_result={cr}")
        return resp

    def exit(self) -> dict:
        """POST /exit: 退出目标区域"""
        rid = self._new_request_id("exit")
        payload = {
            "arena_id": "default",
            "robot_id": self.robot_id,
            "request_id": rid,
        }
        resp = self._post("/exit", payload)
        if resp.get("accepted"):
            print(f"  exit_reason={resp.get('exit_reason')}")
        return resp

    def summary(self) -> list:
        """返回完整日志条目"""
        return self._log_entries
