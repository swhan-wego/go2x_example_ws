#!/usr/bin/env python3
"""
input_publisher.py
─────────────────────────────────────────────────────────────
역할  : 터미널에서 자연어를 입력받아 /user_input 토픽으로 발행하는
        대화형 입력 스크립트

토픽  : Publish  /user_input  (std_msgs/String)

조작:
  - 텍스트 입력 후 Enter : /user_input 발행
  - 'q' 또는 'quit'      : 노드 종료
  - Ctrl+C               : 강제 종료

사용 예시:
  ros2 run llm_robot_agent input_publisher
  # 또는 직접 실행
  python3 input_publisher.py
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InputPublisher(Node):
    def __init__(self):
        super().__init__("input_publisher")

        # ── 파라미터 선언 ──────────────────────────────────────────────────────
        self.declare_parameter("topic", "/user_input")

        topic = self.get_parameter("topic").get_parameter_value().string_value

        # ── ROS2 토픽 설정 ─────────────────────────────────────────────────────
        self.pub = self.create_publisher(String, topic, 10)

        self._should_exit = False

        self.get_logger().info(
            f"Input Publisher 시작 — 발행 토픽: {topic}\n"
            f"  명령 입력 후 Enter → 로봇에 전달\n"
            f"  'q' 또는 'quit' 입력 → 종료"
        )

    def run(self) -> None:
        """터미널 입력 루프 (별도 스레드에서 호출)."""
        print("\n" + "=" * 55)
        print("  LLM 로봇 에이전트 — 자연어 명령 입력기")
        print("=" * 55)
        print("  예시 명령:")
        print("    앞으로 가줘")
        print("    배고픈 강아지처럼 행동해")
        print("    왼쪽으로 90도 돌아")
        print("    멈춰")
        print("=" * 55 + "\n")

        while rclpy.ok() and not self._should_exit:
            try:
                text = input("명령 입력 (종료: q) > ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not text:
                continue

            if text.lower() in ("q", "quit", "exit"):
                self.get_logger().info("사용자 종료 요청")
                self._should_exit = True
                break

            msg = String()
            msg.data = text
            self.pub.publish(msg)
            self.get_logger().info(f"[발행] {text}")


# ── 엔트리포인트 ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = InputPublisher()

    # 입력 루프를 별도 스레드에서 실행 (ROS2 spin과 병렬)
    input_thread = threading.Thread(target=node.run, daemon=True)
    input_thread.start()

    try:
        while rclpy.ok() and not node._should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
