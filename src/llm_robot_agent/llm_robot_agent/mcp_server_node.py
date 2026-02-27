#!/usr/bin/env python3
"""
mcp_server_node.py
─────────────────────────────────────────────────────────────
역할  : /llm_tool_call (JSON 배열) 수신 → Tool 핸들러 디스패치
        → 안전 범위 검증 → /robot_cmd (JSON) 발행

토픽  : Subscribe  /llm_tool_call  (std_msgs/String, JSON 배열)
        Publish    /robot_cmd       (std_msgs/String, JSON 객체)

파라미터:
  - max_speed    : 로봇 최대 선속도 (m/s)   (기본: 0.22)
  - max_duration : 최대 동작 지속 시간 (초) (기본: 10.0)

/robot_cmd JSON 스키마:
  twist 타입 : {"type": "twist", "linear_x": float, "angular_z": float, "duration": float}
  sound 타입 : {"type": "sound", "sound_type": str}
  query 타입 : {"type": "query", "query_type": str, "threshold"?: float}

사용 예시:
  ros2 run llm_robot_agent mcp_server
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ── 안전 상수 ──────────────────────────────────────────────────────────────────
MAX_LINEAR_SPEED  = 0.22   # TurtleBot3 최대 선속도 (m/s)
MAX_ANGULAR_SPEED = 2.84   # TurtleBot3 최대 각속도 (rad/s)
MAX_DURATION      = 10.0   # 최대 동작 지속 시간 (초)
MAX_SPIN_COUNT    = 5      # 최대 spin 반복 횟수
VALID_SOUNDS      = {"bark", "whine", "happy", "sad"}


class MCPServerNode(Node):
    def __init__(self):
        super().__init__("mcp_server_node")

        # ── 파라미터 선언 ──────────────────────────────────────────────────────
        self.declare_parameter("max_speed",    MAX_LINEAR_SPEED)
        self.declare_parameter("max_duration", MAX_DURATION)

        self.max_speed    = float(self.get_parameter(
            "max_speed").get_parameter_value().double_value)
        self.max_duration = float(self.get_parameter(
            "max_duration").get_parameter_value().double_value)

        # ── ROS2 토픽 설정 ─────────────────────────────────────────────────────
        self.sub_tool_call = self.create_subscription(
            String,
            "/llm_tool_call",
            self.tool_call_callback,
            10,
        )
        self.pub_robot_cmd = self.create_publisher(
            String,
            "/robot_cmd",
            10,
        )

        # ── Tool 디스패치 테이블 ───────────────────────────────────────────────
        self.tool_handlers = {
            "move_forward":   self._handle_move_forward,
            "move_backward":  self._handle_move_backward,
            "turn_left":      self._handle_turn_left,
            "turn_right":     self._handle_turn_right,
            "stop":           self._handle_stop,
            "spin":           self._handle_spin,
            "play_sound":     self._handle_play_sound,
            "get_battery":    self._handle_get_battery,
            "check_obstacle": self._handle_check_obstacle,
        }

        self.get_logger().info(
            f"MCP Server Node 시작 — "
            f"max_speed: {self.max_speed} m/s, "
            f"max_duration: {self.max_duration}s"
        )

    # ── 메인 콜백: Tool Call 수신 및 디스패치 ────────────────────────────────
    def tool_call_callback(self, msg: String) -> None:
        try:
            tool_calls = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON 파싱 오류: {e}")
            return

        if not isinstance(tool_calls, list):
            self.get_logger().error("tool_calls가 JSON 배열이 아닙니다.")
            return

        self.get_logger().info(f"[수신] {len(tool_calls)}개 Tool Call")

        for call in tool_calls:
            tool_name = call.get("tool", "")
            args      = call.get("args", {})
            handler   = self.tool_handlers.get(tool_name)

            if handler is None:
                self.get_logger().warn(f"알 수 없는 Tool: '{tool_name}'")
                continue

            try:
                cmd = handler(args)
                if cmd:
                    self._publish_cmd(cmd)
            except Exception as e:
                self.get_logger().error(
                    f"Tool 실행 오류 [{tool_name}]: {e}"
                )

    # ── 명령 발행 헬퍼 ───────────────────────────────────────────────────────
    def _publish_cmd(self, cmd: dict) -> None:
        out = String()
        out.data = json.dumps(cmd, ensure_ascii=False)
        self.pub_robot_cmd.publish(out)
        self.get_logger().info(f"[발행] /robot_cmd: {cmd}")

    # ── Tool 핸들러 ─────────────────────────────────────────────────────────
    def _handle_move_forward(self, args: dict) -> dict:
        speed    = float(args.get("speed",    0.15))
        duration = float(args.get("duration", 2.0))
        # 안전 클리핑
        speed    = max(0.0, min(speed,    self.max_speed))
        duration = max(0.1, min(duration, self.max_duration))
        return {
            "type":      "twist",
            "linear_x":  speed,
            "angular_z": 0.0,
            "duration":  duration,
        }

    def _handle_move_backward(self, args: dict) -> dict:
        speed    = float(args.get("speed",    0.10))
        duration = float(args.get("duration", 2.0))
        speed    = max(0.0, min(speed,    self.max_speed))
        duration = max(0.1, min(duration, self.max_duration))
        return {
            "type":      "twist",
            "linear_x":  -speed,
            "angular_z": 0.0,
            "duration":  duration,
        }

    def _handle_turn_left(self, args: dict) -> dict:
        angle   = float(args.get("angle", 90.0))       # 도 단위
        angle   = max(1.0, min(angle, 360.0))
        ang_spd = 1.0                                   # rad/s 고정
        duration = min(math.radians(angle) / ang_spd, self.max_duration)
        return {
            "type":      "twist",
            "linear_x":  0.0,
            "angular_z": ang_spd,
            "duration":  duration,
        }

    def _handle_turn_right(self, args: dict) -> dict:
        angle   = float(args.get("angle", 90.0))
        angle   = max(1.0, min(angle, 360.0))
        ang_spd = 1.0
        duration = min(math.radians(angle) / ang_spd, self.max_duration)
        return {
            "type":      "twist",
            "linear_x":  0.0,
            "angular_z": -ang_spd,
            "duration":  duration,
        }

    def _handle_stop(self, args: dict) -> dict:
        return {
            "type":      "twist",
            "linear_x":  0.0,
            "angular_z": 0.0,
            "duration":  0.0,
        }

    def _handle_spin(self, args: dict) -> dict:
        speed = float(args.get("speed", 1.5))
        count = int(args.get("count",   2))
        # 안전 클리핑
        speed = max(0.1, min(speed, MAX_ANGULAR_SPEED))
        count = max(1,   min(count, MAX_SPIN_COUNT))
        # 1회전(2π rad) × count 만큼 회전하는 데 필요한 시간
        duration = min((2 * math.pi * count) / speed, self.max_duration)
        return {
            "type":      "twist",
            "linear_x":  0.0,
            "angular_z": speed,
            "duration":  duration,
        }

    def _handle_play_sound(self, args: dict) -> dict:
        sound_type = args.get("sound_type", "bark")
        if sound_type not in VALID_SOUNDS:
            self.get_logger().warn(
                f"알 수 없는 sound_type '{sound_type}' → 'bark'로 대체"
            )
            sound_type = "bark"
        return {
            "type":       "sound",
            "sound_type": sound_type,
        }

    def _handle_get_battery(self, args: dict) -> dict:
        return {
            "type":       "query",
            "query_type": "battery",
        }

    def _handle_check_obstacle(self, args: dict) -> dict:
        threshold = float(args.get("distance_threshold", 0.5))
        threshold = max(0.1, min(threshold, 5.0))
        return {
            "type":       "query",
            "query_type": "obstacle",
            "threshold":  threshold,
        }


# ── 엔트리포인트 ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MCPServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MCP Server Node 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
