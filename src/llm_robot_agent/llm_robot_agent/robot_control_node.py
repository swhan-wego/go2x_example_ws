#!/usr/bin/env python3
"""
robot_control_node.py
─────────────────────────────────────────────────────────────
역할  : /robot_cmd (JSON) 수신 → geometry_msgs/Twist 생성
        → /cmd_vel 주기 퍼블리시 (duration 후 자동 정지)
        → /estop 수신 시 즉시 긴급 정지

토픽  : Subscribe  /robot_cmd  (std_msgs/String, JSON 객체)
                   /estop      (std_msgs/String)
        Publish    /cmd_vel    (geometry_msgs/Twist)

파라미터:
  - cmd_vel_topic : 발행할 cmd_vel 토픽 이름 (기본: /cmd_vel)
  - publish_rate  : Twist 발행 주파수 (Hz)   (기본: 10.0)

/estop 메시지 규칙:
  활성화 : "1" | "true" | "stop"
  해제   : "0" | "false"

사용 예시:
  ros2 run llm_robot_agent robot_control
"""

import json
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class RobotControlNode(Node):
    def __init__(self):
        super().__init__("robot_control_node")

        # ── 파라미터 선언 ──────────────────────────────────────────────────────
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_rate",  10.0)

        self.cmd_vel_topic = self.get_parameter(
            "cmd_vel_topic").get_parameter_value().string_value
        self.publish_rate  = float(self.get_parameter(
            "publish_rate").get_parameter_value().double_value)

        # ── 상태 변수 (스레드 안전) ────────────────────────────────────────────
        self.current_twist = Twist()   # 현재 발행 중인 Twist
        self.stop_time     = 0.0       # 정지 예정 시각 (time.time() 기준)
        self.estop_active  = False     # 긴급 정지 플래그
        self._lock         = threading.Lock()

        # ── QoS: 실시간 제어용 Best-Effort ────────────────────────────────────
        ctrl_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        # ── ROS2 토픽 설정 ─────────────────────────────────────────────────────
        self.sub_robot_cmd = self.create_subscription(
            String,
            "/robot_cmd",
            self.robot_cmd_callback,
            10,
        )
        self.sub_estop = self.create_subscription(
            String,
            "/estop",
            self.estop_callback,
            10,
        )
        self.pub_cmd_vel = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            ctrl_qos,
        )

        # ── 주기 발행 타이머 ───────────────────────────────────────────────────
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback,
        )

        self.get_logger().info(
            f"Robot Control Node 시작 — "
            f"topic: {self.cmd_vel_topic}, "
            f"rate: {self.publish_rate} Hz"
        )

    # ── 타이머 콜백: 주기적 Twist 발행 ───────────────────────────────────────
    def timer_callback(self) -> None:
        with self._lock:
            if self.estop_active:
                # E-Stop 활성화 중 → 항상 정지 Twist
                self.pub_cmd_vel.publish(Twist())
                return

            if time.time() >= self.stop_time:
                # duration 만료 → 정지 Twist
                self.pub_cmd_vel.publish(Twist())
            else:
                # duration 이내 → 현재 Twist 발행
                self.pub_cmd_vel.publish(self.current_twist)

    # ── robot_cmd 수신 콜백 ───────────────────────────────────────────────────
    def robot_cmd_callback(self, msg: String) -> None:
        if self.estop_active:
            self.get_logger().warn("E-Stop 활성화 중 — 명령 무시됨")
            return

        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"robot_cmd JSON 파싱 오류: {e}")
            return

        cmd_type = cmd.get("type", "")

        if   cmd_type == "twist": self._execute_twist(cmd)
        elif cmd_type == "sound": self._execute_sound(cmd)
        elif cmd_type == "query": self._execute_query(cmd)
        else:
            self.get_logger().warn(f"알 수 없는 cmd type: '{cmd_type}'")

    # ── 긴급 정지 콜백 ───────────────────────────────────────────────────────
    def estop_callback(self, msg: String) -> None:
        value = msg.data.strip().lower()
        with self._lock:
            if value in ("1", "true", "stop"):
                self.estop_active = True
                self.pub_cmd_vel.publish(Twist())
                self.get_logger().warn("⚠️  긴급 정지(E-Stop) 발동!")
            elif value in ("0", "false"):
                self.estop_active = False
                self.get_logger().info("✅ E-Stop 해제")
            else:
                self.get_logger().warn(f"알 수 없는 /estop 값: '{value}'")

    # ── Twist 명령 실행 ──────────────────────────────────────────────────────
    def _execute_twist(self, cmd: dict) -> None:
        twist           = Twist()
        duration        = float(cmd.get("duration",  0.0))
        twist.linear.x  = float(cmd.get("linear_x",  0.0))
        twist.linear.y  = 0.0
        twist.linear.z  = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(cmd.get("angular_z", 0.0))

        with self._lock:
            if duration <= 0.0:
                # duration == 0 → 즉시 정지
                self.current_twist = Twist()
                self.stop_time     = time.time()
            else:
                self.current_twist = twist
                self.stop_time     = time.time() + duration

        self.get_logger().info(
            f"[Twist] linear_x={twist.linear.x:.2f} m/s  "
            f"angular_z={twist.angular.z:.2f} rad/s  "
            f"duration={duration:.1f}s"
        )

    # ── 소리 명령 실행 ───────────────────────────────────────────────────────
    def _execute_sound(self, cmd: dict) -> None:
        sound_type = cmd.get("sound_type", "bark")
        # 실제 로봇에 스피커가 있다면 해당 인터페이스 호출
        # 예: os.system(f"aplay /sounds/{sound_type}.wav")
        self.get_logger().info(f"🔊 소리 재생: {sound_type}")

    # ── 쿼리 명령 실행 ───────────────────────────────────────────────────────
    def _execute_query(self, cmd: dict) -> None:
        query_type = cmd.get("query_type", "")

        if query_type == "battery":
            # 실제 구현 시: 배터리 토픽 구독값을 읽어 반환
            self.get_logger().info("🔋 배터리 잔량 조회 요청 수신")

        elif query_type == "obstacle":
            threshold = cmd.get("threshold", 0.5)
            # 실제 구현 시: LiDAR 토픽 구독값과 비교
            self.get_logger().info(
                f"📡 장애물 감지 조회 — 임계 거리: {threshold}m"
            )

        else:
            self.get_logger().warn(f"알 수 없는 query_type: '{query_type}'")


# ── 엔트리포인트 ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Robot Control Node 종료")
    finally:
        # 종료 시 반드시 정지 Twist 발행
        node.pub_cmd_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
