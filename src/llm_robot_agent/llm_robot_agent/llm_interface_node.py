#!/usr/bin/env python3
"""
llm_interface_node.py
─────────────────────────────────────────────────────────────
역할  : 사용자 자연어 입력 수신 → Ollama LLM 호출
        → Tool Call JSON 파싱 → /llm_tool_call 발행

토픽  : Subscribe  /user_input    (std_msgs/String)
        Publish    /llm_tool_call  (std_msgs/String, JSON 배열)

파라미터:
  - model_name  : Ollama 모델 이름        (기본: llama3.1:8b)
  - ollama_host : Ollama 서버 주소        (기본: http://localhost:11434)
  - max_history : 대화 이력 최대 턴 수   (기본: 10)

사용 예시:
  ros2 run llm_robot_agent llm_interface
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama


# ── 시스템 프롬프트 ────────────────────────────────────────────────────────────
SYSTEM_PROMPT = """당신은 로봇 제어 AI 에이전트입니다.
사용자의 자연어 명령을 분석하여 반드시 아래 Tool 중 하나 이상을 호출하세요.

사용 가능한 Tool:
  move_forward(speed: float, duration: float)   앞으로 이동 (speed: 0.0~0.22 m/s)
  move_backward(speed: float, duration: float)  뒤로 이동
  turn_left(angle: float)                       좌회전 (angle: 도 단위)
  turn_right(angle: float)                      우회전
  stop()                                        즉시 정지
  spin(speed: float, count: int)                제자리 회전 반복
  play_sound(sound_type: str)                   소리 재생
    sound_type: "bark" | "whine" | "happy" | "sad"
  get_battery()                                 배터리 잔량 조회
  check_obstacle(distance_threshold: float)     전방 장애물 감지

반드시 JSON 배열 형식으로만 응답하세요. 예시:
[{"tool": "move_forward", "args": {"speed": 0.15, "duration": 2.0}}]
설명 텍스트 없이 JSON만 출력하세요."""


class LLMInterfaceNode(Node):
    def __init__(self):
        super().__init__("llm_interface_node")

        # ── 파라미터 선언 ──────────────────────────────────────────────────────
        self.declare_parameter("model_name",  "llama3.1:8b")
        self.declare_parameter("ollama_host", "http://localhost:11434")
        self.declare_parameter("max_history", 10)

        self.model_name  = self.get_parameter(
            "model_name").get_parameter_value().string_value
        self.ollama_host = self.get_parameter(
            "ollama_host").get_parameter_value().string_value
        self.max_history = int(self.get_parameter(
            "max_history").get_parameter_value().integer_value)

        # ── 대화 이력 초기화 ───────────────────────────────────────────────────
        self.conversation_history = []

        # ── Ollama 클라이언트 초기화 ───────────────────────────────────────────
        self.ollama_client = ollama.Client(host=self.ollama_host)

        # ── ROS2 토픽 설정 ─────────────────────────────────────────────────────
        self.sub_user_input = self.create_subscription(
            String,
            "/user_input",
            self.user_input_callback,
            10,
        )
        self.pub_tool_call = self.create_publisher(
            String,
            "/llm_tool_call",
            10,
        )

        self.get_logger().info(
            f"LLM Interface Node 시작 — 모델: {self.model_name}, "
            f"host: {self.ollama_host}, max_history: {self.max_history}"
        )

    # ── 콜백: 사용자 입력 수신 ────────────────────────────────────────────────
    def user_input_callback(self, msg: String) -> None:
        user_text = msg.data.strip()
        if not user_text:
            return

        self.get_logger().info(f"[입력] {user_text}")

        # 대화 이력에 사용자 메시지 추가
        self.conversation_history.append(
            {"role": "user", "content": user_text}
        )

        # 이력 길이 제한 (max_history 턴 × 2 메시지)
        if len(self.conversation_history) > self.max_history * 2:
            self.conversation_history = \
                self.conversation_history[-self.max_history * 2:]

        try:
            tool_calls_json = self._call_llm()
            self._publish_tool_calls(tool_calls_json)
        except Exception as e:
            self.get_logger().error(f"LLM 호출 실패: {e}")

    # ── Ollama API 호출 ───────────────────────────────────────────────────────
    def _call_llm(self) -> str:
        """Ollama API를 호출하고 Tool Call JSON 문자열을 반환합니다."""
        messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        messages += self.conversation_history

        response = self.ollama_client.chat(
            model=self.model_name,
            messages=messages,
        )
        reply = response["message"]["content"].strip()

        # 어시스턴트 응답 이력 추가
        self.conversation_history.append(
            {"role": "assistant", "content": reply}
        )

        self.get_logger().info(f"[LLM 응답] {reply}")
        return reply

    # ── Tool Call 발행 ────────────────────────────────────────────────────────
    def _publish_tool_calls(self, tool_calls_json: str) -> None:
        """JSON 유효성 검증 후 /llm_tool_call 토픽으로 발행합니다."""
        try:
            parsed = json.loads(tool_calls_json)
            if not isinstance(parsed, list):
                raise ValueError("응답이 JSON 배열이 아닙니다.")
        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().warn(f"JSON 파싱 실패 — skip: {e}")
            return

        out_msg = String()
        out_msg.data = tool_calls_json
        self.pub_tool_call.publish(out_msg)
        self.get_logger().info(
            f"[발행] /llm_tool_call — {len(parsed)}개 Tool Call"
        )


# ── 엔트리포인트 ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = LLMInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("LLM Interface Node 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
