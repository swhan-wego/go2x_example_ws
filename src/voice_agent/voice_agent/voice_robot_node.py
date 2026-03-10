"""voice_robot_node.py — 음성 명령으로 로봇을 제어하는 ROS2 노드

webrtc_unitree_ros.py 가 발행하는 토픽을 통해 로봇을 제어합니다.

구독 토픽 (없음 — 완전히 발행 전용)
발행 토픽:
  /cmd_vel    (geometry_msgs/Twist)  — 이동 명령
  /build_cmd  (std_msgs/String)      — 동작 명령 (JSON)

실행 예시:
  ros2 run voice_agent voice_robot_node
"""

import json
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from voice_agent.command_parser import parse_command
from voice_agent.stt_module import STTEngine
from voice_agent.tts_module import TTSEngine
from voice_agent.wakeword_detector import WakeWordDetector


# ── 이동 파라미터 ──────────────────────────────────────────────────────────────
LINEAR_SPEED = 0.3  # m/s
ANGULAR_SPEED = 0.5  # rad/s
RECORD_DURATION = 3.0  # 녹음 시간 (초)

# ── 명령 → 토픽 매핑 ──────────────────────────────────────────────────────────
#   cmd_vel  : Twist 메시지로 전달
#   build_cmd: JSON 문자열로 전달 (COMMAND_MAP 키 사용)


def _make_twist(vx: float = 0.0, vy: float = 0.0, vz: float = 0.0) -> Twist:
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = vz
    return msg


# command_parser 키 → (토픽 종류, 메시지 생성 함수)
COMMAND_ACTIONS: dict[str, tuple[str, callable]] = {
    "forward": ("cmd_vel", lambda: _make_twist(vx=LINEAR_SPEED)),
    "backward": ("cmd_vel", lambda: _make_twist(vx=-LINEAR_SPEED)),
    "turn_left": ("cmd_vel", lambda: _make_twist(vz=ANGULAR_SPEED)),
    "turn_right": ("cmd_vel", lambda: _make_twist(vz=-ANGULAR_SPEED)),
    "stop": ("cmd_vel", lambda: _make_twist()),
    "sit": ("build_cmd", lambda: json.dumps({"type": "sit"})),
    "stand": ("build_cmd", lambda: json.dumps({"type": "stand_up"})),
    "down": ("build_cmd", lambda: json.dumps({"type": "stand_down"})),
}


class VoiceRobotNode(Node):
    """음성 명령 → ROS2 토픽 발행 노드"""

    def __init__(self):
        super().__init__("voice_robot_node")
        self._log = self.get_logger()

        # 파라미터
        self.declare_parameter("record_duration", RECORD_DURATION)
        self.declare_parameter("stt_model", "small")
        self.declare_parameter("linear_speed", LINEAR_SPEED)
        self.declare_parameter("angular_speed", ANGULAR_SPEED)

        self._record_duration = (
            self.get_parameter("record_duration").get_parameter_value().double_value
        )
        stt_model = self.get_parameter("stt_model").get_parameter_value().string_value
        self._linear_speed = (
            self.get_parameter("linear_speed").get_parameter_value().double_value
        )
        self._angular_speed = (
            self.get_parameter("angular_speed").get_parameter_value().double_value
        )

        # 발행 토픽
        self._cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._build_cmd_pub = self.create_publisher(String, "build_cmd", 10)

        # STT / TTS / WakeWord 초기화
        self._log.info(f"STT 모델 로딩 중 (model={stt_model})...")
        self._stt = STTEngine(model_size=stt_model)
        self._tts = TTSEngine()
        self._wakeword = WakeWordDetector(model=self._stt.model)
        self._log.info("STT/TTS/WakeWord 초기화 완료")

        # 음성 인식 루프를 별도 스레드에서 실행 (ROS spin 블로킹 방지)
        self._running = True
        self._voice_thread = threading.Thread(target=self._voice_loop, daemon=True)
        self._voice_thread.start()

        self._log.info("VoiceRobotNode 준비 완료 — '고투야'라고 말해주세요!")

    # ── 내부 메서드 ────────────────────────────────────────────────────────────

    def _voice_loop(self):
        """웨이크워드 대기 → STT → 명령 파싱 → 토픽 발행을 반복하는 루프"""
        while self._running and rclpy.ok():
            try:
                self._log.info("대기 중... ('고투야'라고 말해주세요)")
                self._wakeword.wait_for_wake_word()
                self._log.info(
                    f"웨이크워드 감지! [{self._record_duration:.0f}초] 명령을 말씀해 주세요..."
                )
                text = self._stt.listen(self._record_duration)
                self._log.info(f"인식 결과: '{text}'")

                if not text:
                    # self._tts.speak("음성을 인식하지 못했습니다.")
                    continue

                command, feedback = parse_command(text)

                if command is None:
                    # self._tts.speak("알 수 없는 명령입니다.")
                    self._log.warn(f"알 수 없는 명령 텍스트: '{text}'")
                    continue

                # 로봇 제어 토픽 발행
                self._publish_command(command)

                # TTS 피드백 완료 후 다음 녹음 시작
                self._tts.speak(feedback, blocking=True)

            except Exception as e:
                self._log.error(f"음성 루프 오류: {e}")

    def _publish_command(self, command: str):
        """command_key에 따라 cmd_vel 또는 build_cmd 토픽 발행"""
        action = COMMAND_ACTIONS.get(command)
        if action is None:
            self._log.warning(f"매핑되지 않은 명령: {command}")
            return

        topic_type, msg_factory = action

        if topic_type == "cmd_vel":
            twist_msg: Twist = msg_factory()
            # 파라미터로 받은 속도값 반영
            if twist_msg.linear.x != 0.0:
                twist_msg.linear.x = (
                    self._linear_speed
                    if twist_msg.linear.x > 0
                    else -self._linear_speed
                )
            if twist_msg.angular.z != 0.0:
                twist_msg.angular.z = (
                    self._angular_speed
                    if twist_msg.angular.z > 0
                    else -self._angular_speed
                )
            self._cmd_vel_pub.publish(twist_msg)
            self._log.info(
                f"[cmd_vel] linear.x={twist_msg.linear.x:.2f} "
                f"linear.y={twist_msg.linear.y:.2f} "
                f"angular.z={twist_msg.angular.z:.2f}"
            )

        elif topic_type == "build_cmd":
            json_str: str = msg_factory()
            str_msg = String()
            str_msg.data = json_str
            self._build_cmd_pub.publish(str_msg)
            self._log.info(f"[build_cmd] {json_str}")

    def destroy_node(self):
        self._running = False
        super().destroy_node()


# ── 엔트리포인트 ───────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = VoiceRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
