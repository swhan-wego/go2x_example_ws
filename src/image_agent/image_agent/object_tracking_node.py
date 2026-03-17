"""object_tracking_node.py — /object_position 토픽을 받아 로봇이 물체를 추종하는 ROS2 노드.

/object_position (geometry_msgs/PointStamped) 의 정규화 좌표를 기반으로
cmd_vel (geometry_msgs/Twist) 를 발행해 로봇을 제어합니다.

추적 절차:
  1. [ALIGN]    X축 오차(|x - 0.5|) 가 x_tolerance 이내가 될 때까지 제자리 회전
  2. [APPROACH] X가 정렬되면 전진. y > (1 - y_stop_range) 이면 정지(도착)
  3.            전진 중 X 오차가 x_tolerance 초과 시 1번으로 복귀

좌표 규약 (object_yolo_node 기준):
  x: 0.0(좌) ~ 1.0(우),  0.5 = 화면 중앙
  y: 0.0(상) ~ 1.0(하),  1.0 = 화면 하단 (= 물체가 가까움)
  z: 검출 신뢰도

서비스:
  ~/enable  (std_srvs/srv/SetBool)  — True: 추적 시작 / False: 추적 중단

파라미터:
  object_pos_topic (str,   기본 "/object_position") — 위치 입력 토픽
  cmd_vel_topic    (str,   기본 "/cmd_vel")          — 이동 명령 출력 토픽
  x_tolerance      (float, 기본 0.1,  범위 0.01~0.5) — X축 정렬 허용 오차
  y_stop_range     (float, 기본 0.25, 범위 0.01~0.5) — 전진 정지 Y 임계값
                          y > (1 - y_stop_range) 이면 도달 판정
  timer_period     (float, 기본 0.1,  범위 0.01~1.0) — 제어 루프 주기 (초)
  max_speed        (float, 기본 0.3,  범위 0.01~1.0) — 최대 이동/회전 속도
  pos_timeout      (float, 기본 1.0)  — 이 시간(초) 동안 위치 수신 없으면 정지
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from std_srvs.srv import SetBool

# 추적 상태
_ALIGN   = "ALIGN"    # X축 정렬 중
_APPROACH = "APPROACH" # 전진 중
_ARRIVED  = "ARRIVED"  # 도달 (정지)


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


class ObjectTrackingNode(Node):
    """물체 위치 기반 로봇 추종 노드."""

    def __init__(self):
        super().__init__("object_tracking_node")

        # ── 파라미터 ──────────────────────────────────────────────────────
        self.declare_parameter("object_pos_topic", "/object_position")
        self.declare_parameter("cmd_vel_topic",    "/cmd_vel")
        self.declare_parameter("x_tolerance",      0.1)
        self.declare_parameter("y_stop_range",     0.25)
        self.declare_parameter("timer_period",     0.1)
        self.declare_parameter("max_speed",        0.3)
        self.declare_parameter("pos_timeout",      1.0)

        object_pos_topic = self.get_parameter("object_pos_topic").value
        cmd_vel_topic    = self.get_parameter("cmd_vel_topic").value
        self._x_tol      = float(self.get_parameter("x_tolerance").value)
        self._y_stop     = float(self.get_parameter("y_stop_range").value)
        self._period     = float(self.get_parameter("timer_period").value)
        self._max_spd    = float(self.get_parameter("max_speed").value)
        self._pos_timeout = float(self.get_parameter("pos_timeout").value)

        # 파라미터 범위 보정
        self._x_tol  = max(0.01, min(0.5,  self._x_tol))
        self._y_stop = max(0.01, min(0.5,  self._y_stop))
        self._period = max(0.01, min(1.0,  self._period))
        self._max_spd = max(0.01, min(1.0, self._max_spd))

        # ── 상태 ──────────────────────────────────────────────────────────
        self._tracking   = False
        self._state      = _ALIGN
        self._latest_pos: PointStamped | None = None
        self._timer      = None  # 제어 루프 타이머 (추적 중에만 활성)

        # ── 퍼블리셔 / 구독자 ─────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(
            PointStamped, object_pos_topic, self._pos_callback, 10
        )

        # ── 서비스 ────────────────────────────────────────────────────────
        self.create_service(SetBool, "~/enable", self._enable_callback)

        self.get_logger().info(
            f"ObjectTrackingNode 준비 | pos={object_pos_topic} cmd={cmd_vel_topic}"
            f" | x_tol={self._x_tol} y_stop={self._y_stop}"
            f" | max_speed={self._max_spd} period={self._period}s"
        )

    # ── 콜백 ──────────────────────────────────────────────────────────────

    def _pos_callback(self, msg: PointStamped):
        """최신 위치만 저장."""
        self._latest_pos = msg

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self._start_tracking()
            response.success = True
            response.message = "추적 시작"
        else:
            self._stop_tracking()
            response.success = True
            response.message = "추적 중단"
        return response

    # ── 추적 제어 ─────────────────────────────────────────────────────────

    def _start_tracking(self):
        if self._tracking:
            return
        self._tracking = True
        self._state    = _ALIGN
        self._timer    = self.create_timer(self._period, self._control_loop)
        self.get_logger().info("추적 ON")

    def _stop_tracking(self):
        if not self._tracking:
            return
        self._tracking = False
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self._publish_stop()
        self.get_logger().info("추적 OFF")

    def _publish_stop(self):
        self._cmd_pub.publish(Twist())

    # ── 제어 루프 ─────────────────────────────────────────────────────────

    def _control_loop(self):
        pos = self._latest_pos

        # 위치 데이터 타임아웃 확인
        if pos is None:
            self._publish_stop()
            return

        now = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(pos.header.stamp)
        if msg_time.nanoseconds > 0:
            age = (now - msg_time).nanoseconds * 1e-9
            if age > self._pos_timeout:
                self._publish_stop()
                return

        x = pos.point.x  # 0.0(좌) ~ 1.0(우)
        y = pos.point.y  # 0.0(상) ~ 1.0(하)

        x_err = x - 0.5  # 양수 = 오른쪽, 음수 = 왼쪽

        twist = Twist()

        if self._state == _ALIGN:
            if abs(x_err) <= self._x_tol:
                # 정렬 완료 → 전진 단계로
                self._state = _APPROACH
                self.get_logger().debug("상태: APPROACH")
            else:
                # 제자리 회전 (비례 제어)
                # x가 오른쪽이면(x_err > 0) 오른쪽 회전 → angular.z 음수
                twist.angular.z = _clamp(-x_err * 2.0, self._max_spd)

        elif self._state == _APPROACH:
            if abs(x_err) > self._x_tol:
                # X 오차 재발 → 정렬 단계로 복귀
                self._state = _ALIGN
                self.get_logger().debug("상태: ALIGN (X 오차 재발)")
            elif y >= (1.0 - self._y_stop):
                # 목표 도달 → 정지
                self._state = _ARRIVED
                self.get_logger().info("목표 도달. 정지.")
            else:
                # 전진 + 미세 조향
                twist.linear.x  = self._max_spd
                twist.angular.z = _clamp(-x_err * 2.0, self._max_spd * 0.5)

        elif self._state == _ARRIVED:
            # 도달 상태: 물체가 멀어지면 다시 정렬 시작
            if y < (1.0 - self._y_stop) or abs(x_err) > self._x_tol:
                self._state = _ALIGN
                self.get_logger().debug("상태: ALIGN (물체 이탈)")

        self._cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
