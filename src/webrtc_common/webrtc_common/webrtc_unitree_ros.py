"""WebRTC 데이터 토픽과 ROS2 토픽을 중계하는 노드

ros2 run <package> <executable> --ros-args --params-file <path_to_params_file>

웹캠 영상 중계
키보드 핸들러

메인 쓰레드
- ROS2 node spin

보조 쓰레드
- WebRTC ASYNC 처리

webrtc cam -> queue -> ros2 topic pub

webrtc conn ok -> event -> ros timer start
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist  # 추가
import asyncio
import threading
import logging
import json
import janus
from queue import Queue
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

logging.getLogger("unitree_webrtc_connect").setLevel(logging.INFO)

COMMAND_MAP = {
    "move": lambda cmd: (
        RTC_TOPIC["SPORT_MOD"],
        {
            "api_id": SPORT_CMD["Move"],
            "parameter": {"x": cmd["x"], "y": cmd["y"], "z": cmd["z"]},
        },
    ),
    "euler": lambda cmd: (
        RTC_TOPIC["SPORT_MOD"],
        {
            "api_id": SPORT_CMD["Euler"],
            "parameter": {"x": cmd["roll"], "y": cmd["pitch"], "z": cmd["yaw"]},
        },
    ),
    "bodyheight": lambda cmd: (
        RTC_TOPIC["SPORT_MOD"],
        {"api_id": SPORT_CMD["BodyHeight"], "parameter": {"z": cmd["height"]}},
    ),
    "balance_stand": lambda cmd: (
        RTC_TOPIC["SPORT_MOD"],
        {"api_id": SPORT_CMD["BalanceStand"]},
    ),
    "stand_down": lambda cmd: (
        RTC_TOPIC["SPORT_MOD"],
        {"api_id": SPORT_CMD["StandDown"]},
    ),
    "stand_up": lambda cmd: (RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandUp"]}),
    "damp": lambda cmd: (RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["Damp"]}),
    "hello": lambda cmd: (RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["Hello"]}),
    "sit": lambda cmd: (RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["Sit"]}),
    "pose": lambda cmd: (
        RTC_TOPIC["SPORT_MOD"],
        {"api_id": SPORT_CMD["Pose"], "parameter": {"flag": True}},
    ),
}


class UnitreeRobotNode(Node):
    def __init__(self):
        super().__init__("unitree_robot_node")
        self._log = self.get_logger()

        # 파라미터 선언
        self.declare_parameter("ip", "")
        self.declare_parameter("is_hotspot", False)
        # 이동 속도 파라미터 (기본값은 기존 상수와 동일)
        self.declare_parameter("move_speed", 0.5)
        self.declare_parameter("side_speed", 0.3)
        self.declare_parameter("turn_speed", 1.0)

        self.ip = self.get_parameter("ip").get_parameter_value().string_value
        self.is_hotspot = (
            self.get_parameter("is_hotspot").get_parameter_value().bool_value
        )
        # 파라미터에서 속도값 읽기
        self.move_speed = (
            self.get_parameter("move_speed").get_parameter_value().double_value
        )
        self.side_speed = (
            self.get_parameter("side_speed").get_parameter_value().double_value
        )
        self.turn_speed = (
            self.get_parameter("turn_speed").get_parameter_value().double_value
        )

        # 명령 큐
        self.command_queue: janus.Queue[dict] = janus.Queue()

        # 카메라 프레임 큐
        self.frame_queue = janus.Queue()

        # 토픽 구독 설정
        self.pressed_sub = self.create_subscription(
            Int32, "keyboard_pressed", self.pressed_callback, 10
        )
        self.released_sub = self.create_subscription(
            Int32, "keyboard_released", self.released_callback, 10
        )
        # 외부 노드에서 JSON 명령을 직접 전송할 수 있는 토픽
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )
        # COMMAND_MAP 키를 직접 호출하는 토픽
        # 포맷: {"type": "move", "x": 0.5, "y": 0.0, "z": 0.0}
        self.build_cmd_sub = self.create_subscription(
            String, "build_cmd", self.build_cmd_callback, 10
        )

        # 발행 토픽 설정
        self.frontcam_pub = self.create_publisher(Image, "front_camera", 10)

        # WebRTC 연결 후 영상 토픽 발행
        self.timer_connected = self.create_timer(0.5, self.wait_connected)

        # WebRTC 연결 설정
        if self.ip != "":
            self.conn = UnitreeWebRTCConnection(
                WebRTCConnectionMethod.LocalSTA, ip=self.ip
            )
            logging.info(f"Connecting to STA mode with IP: {self.ip}")
        else:
            self.conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)
            logging.info("Connecting to AP mode")

        # 속도 상태
        self.vx, self.vy, self.vz = 0.0, 0.0, 0.0
        self.is_standing = False

        # 자세 상태
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.body_height = 0.0

        # 시그널
        self._connected_signal = threading.Event()

        self.bridge = CvBridge()

        # WebRTC 루프를 별도 스레드에서 실행
        self.loop = asyncio.new_event_loop()
        self.asyncio_thread = threading.Thread(
            target=self.run_asyncio_loop, daemon=True
        )
        self.asyncio_thread.start()

    def wait_connected(self):
        # self._log.info("Checking something...")
        if self._connected_signal.is_set():
            self.timer_connected.cancel()
            self.timer_frontcam = self.create_timer(0.03, self.received_frontcam)

    def received_frontcam(self):
        # self._log.info("Publishing front camera frame")
        if not self.frame_queue.sync_q.empty():
            frame = self.frame_queue.sync_q.get()
            ros_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.frontcam_pub.publish(ros_img)

    async def send_command(self, item):
        try:
            topic = item["topic"]
            payload = item["payload"]
            await self.conn.datachannel.pub_sub.publish_request_new(topic, payload)
        except Exception as e:
            self._log.error(f"Error sending command: {e}")

    def _build_and_enqueue(self, cmd: dict) -> bool:
        """cmd dict를 COMMAND_MAP으로 변환해 topic/payload 포함 항목을 큐에 넣는다.
        성공 시 True, 알 수 없는 타입이면 False 반환."""
        builder = COMMAND_MAP.get(cmd["type"])
        if builder is None:
            self._log.warning(f"알 수 없는 명령 타입: {cmd['type']}")
            return False
        topic, payload = builder(cmd)
        self.command_queue.sync_q.put({"topic": topic, "payload": payload})
        return True

    def pressed_callback(self, msg):
        key_val = msg.data
        self._log.info(f"Key pressed: {key_val}")
        try:
            char = chr(key_val)
        except ValueError:
            char = None

        self.vx, self.vy, self.vz = 0, 0, 0

        move_flag = False
        euler_flag = False
        height_flag = False

        if key_val == 32:  # Space
            self.yaw, self.roll, self.pitch, self.body_height = 0.0, 0.0, 0.0, 0.0
            self.vx, self.vy, self.vz, move_flag = 0.0, 0.0, 0.0, True
        else:
            match char:
                case "w":
                    self.vx, move_flag = self.move_speed, True
                case "s":
                    self.vx, move_flag = -self.move_speed, True
                case "a":
                    self.vy, move_flag = self.side_speed, True
                case "d":
                    self.vy, move_flag = -self.side_speed, True
                case "q":
                    self.vz, move_flag = self.turn_speed, True
                case "e":
                    self.vz, move_flag = -self.turn_speed, True
                case "r":
                    self.roll, euler_flag = self.roll + 0.1, True
                case "f":
                    self.roll, euler_flag = self.roll - 0.1, True
                case "t":
                    self.pitch, euler_flag = self.pitch + 0.1, True
                case "g":
                    self.pitch, euler_flag = self.pitch - 0.1, True
                case "y":
                    self.yaw, euler_flag = self.yaw + 0.1, True
                case "h":
                    self.yaw, euler_flag = self.yaw - 0.1, True
                case "u":
                    self.body_height, height_flag = self.body_height + 0.03, True
                case "j":
                    self.body_height, height_flag = self.body_height - 0.03, True
                case "1":
                    self._build_and_enqueue({"type": "damp"})
                case "2":
                    self._build_and_enqueue({"type": "hello"})
                case "3":
                    self._build_and_enqueue({"type": "sit"})
                case "v":
                    self._build_and_enqueue({"type": "pose"})
                case "z":
                    self._build_and_enqueue({"type": "stand_down"})
                case "x":
                    self._build_and_enqueue({"type": "stand_up"})
                case "c":
                    self._build_and_enqueue({"type": "balance_stand"})

        if move_flag:
            self._build_and_enqueue(
                {"type": "move", "x": self.vx, "y": self.vy, "z": self.vz}
            )
        if euler_flag:
            self.roll = max(min(self.roll, 0.75), -0.75)
            self.pitch = max(min(self.pitch, 0.75), -0.75)
            self.yaw = max(min(self.yaw, 0.6), -0.6)
            self._build_and_enqueue(
                {
                    "type": "euler",
                    "roll": self.roll,
                    "pitch": self.pitch,
                    "yaw": self.yaw,
                }
            )
        if height_flag:
            self.body_height = max(min(self.body_height, 0.03), -0.18)
            self._build_and_enqueue({"type": "bodyheight", "height": self.body_height})

    def released_callback(self, msg):
        pass

    def cmd_vel_callback(self, msg: Twist) -> None:
        """표준 cmd_vel 토픽 콜백.
        linear.x → 전진/후진
        linear.y → 좌/우 이동
        angular.z → 회전
        """
        self._build_and_enqueue({
            "type": "move",
            "x": float(msg.linear.x),
            "y": float(msg.linear.y),
            "z": float(msg.angular.z),
        })
        self._log.info(
            f"[cmd_vel] linear=({msg.linear.x:.2f}, {msg.linear.y:.2f}) "
            f"angular.z={msg.angular.z:.2f}"
        )

    def build_cmd_callback(self, msg: String) -> None:
        """COMMAND_MAP 키를 직접 호출하는 토픽 콜백.

        포맷 예시:
          move        : {"type": "move", "x": 0.5, "y": 0.0, "z": 0.0}
          euler       : {"type": "euler", "roll": 0.1, "pitch": 0.0, "yaw": 0.0}
          bodyheight  : {"type": "bodyheight", "height": 0.03}
          stand_up    : {"type": "stand_up"}
          stand_down  : {"type": "stand_down"}
          balance_stand: {"type": "balance_stand"}
          damp        : {"type": "damp"}
          hello       : {"type": "hello"}
          sit         : {"type": "sit"}
          pose        : {"type": "pose"}
        """
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._log.error(f"build_cmd JSON 파싱 오류: {e}")
            return

        if "type" not in cmd:
            self._log.error("build_cmd: 'type' 필드 누락")
            return

        success = self._build_and_enqueue(cmd)
        if success:
            self._log.info(f"[build_cmd] 명령 수신: {cmd}")

    async def recv_camera_stream(self, track):
        fail_count = 0
        while True:
            try:
                frame = await track.recv()
                img = frame.to_ndarray(format="bgr24")

                # 큐가 가득 찬 경우 오래된 것을 명시적으로 제거
                while not self.frame_queue.async_q.empty():
                    try:
                        self.frame_queue.async_q.get_nowait()
                    except asyncio.QueueEmpty:
                        break

                await self.frame_queue.async_q.put(img)
                fail_count = 0

            except Exception as e:
                fail_count += 1
                self._log.warn(f"[Video] 프레임 수신 실패 ({fail_count}/30): {e}")
                if fail_count > 30:
                    self._log.error("[Video] 영상 스트림 유지 실패, 종료")
                    break
                await asyncio.sleep(0.03)

    def run_asyncio_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.setup_and_control())

    async def setup_and_control(self):
        self._log.info("Connecting to robot via WebRTC...")
        await self.conn.connect()
        self._log.info("Connected to robot!")
        self.conn.video.switchVideoChannel(True)
        self.conn.video.add_track_callback(self.recv_camera_stream)

        self._connected_signal.set()

        # 모드 전환 (Normal 모드로)
        try:
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"],
                {"api_id": 1002, "parameter": {"name": "normal"}},
            )
            await asyncio.sleep(2)
            self._log.info("Switched to 'normal' mode.")
        except Exception as e:
            self._log.error(f"Failed to switch mode: {e}")

        # 제어 루프
        while rclpy.ok():
            try:
                cmd = await asyncio.wait_for(
                    self.command_queue.async_q.get(),
                    timeout=0.05,  # 0.05초 내 명령 없으면 루프 순환
                )
                await self.send_command(cmd)
            except asyncio.TimeoutError:
                pass  # 명령 없음, 다음 순환


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # janus.Queue 종료 (aclose는 코루틴이므로 루프에서 실행)
        node.loop.call_soon_threadsafe(node.loop.create_task, node._cleanup())
        node.asyncio_thread.join(timeout=3.0)  # asyncio Thread 종료 대기
        node.destroy_node()
        rclpy.shutdown()


async def _cleanup(self):
    await self.command_queue.aclose()
    await self.frame_queue.aclose()
    self.loop.stop()


if __name__ == "__main__":
    main()
