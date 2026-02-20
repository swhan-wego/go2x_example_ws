"""QRCODE 찾기 게임 노드

간단 사용법:
  python3 webrtc_ros_qrgame.py

파라미터:
  - image_topic: 구독할 이미지 토픽 (기본: front_camera)
  - score_max: 목표 점수 (기본: 5)
  - game_time: 게임 지속 시간(초, 기본: 60)

조작:
    - 키 입력은 `keyboard_handler.py`를 통해 받습니다 (예: 'i' 시작, 'o' 리셋, 'p' 종료).

노드는 카메라 프레임을 받아 화면에 표시하고, `qrcode.py`가 발행하는 QR 결과 토픽을 통해 점수를 집계합니다.
"""

import os
import time
from typing import Optional, List

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
import numpy as np
from cv_bridge import CvBridge


class QRGameNode(Node):
    def __init__(self):
        super().__init__("qr_game_node")

        # parameters
        self.declare_parameter("image_topic", "front_camera")
        self.declare_parameter("score_max", 5)
        self.declare_parameter("game_time", 60.0)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.score_max = int(self.get_parameter("score_max").get_parameter_value().integer_value)
        self.game_time = float(self.get_parameter("game_time").get_parameter_value().double_value)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.found = []  # ordered found texts
        self.found_set = set()

        self.state = "waiting"  # waiting, running, finished
        self.start_time = 0.0

        # subscribe to camera frames (drop older frames at transport, keep only latest)
        camera_qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.image_topic, self.image_callback, camera_qos)

        # subscribe to QR code results published by `qrcode.py` (std_msgs/String)
        self.declare_parameter("qrcode_topic", "/qrcode")
        self.qrcode_topic = self.get_parameter("qrcode_topic").get_parameter_value().string_value
        self.create_subscription(String, self.qrcode_topic, self.qrcode_callback, 10)

        # subscribe to keyboard events published by `keyboard_handler.py`
        self.declare_parameter("keyboard_pressed_topic", "keyboard_pressed")
        self.keyboard_pressed_topic = self.get_parameter("keyboard_pressed_topic").get_parameter_value().string_value
        self.create_subscription(Int32, self.keyboard_pressed_topic, self.keyboard_pressed_callback, 10)

        self._should_exit = False

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def qrcode_callback(self, msg: String) -> None:
        """Receive QR detection results from `qrcode.py` via topic and update score."""
        try:
            text = msg.data
            if not text:
                return
            # only accept new codes while game is running
            if self.state != "running":
                return
            if text not in self.found_set:
                self.found.append(text)
                self.found_set.add(text)
                self.get_logger().info(f"Found QR (via topic): {text} (score {len(self.found)}/{self.score_max})")
                if len(self.found) >= self.score_max:
                    self.state = "finished"
                    self.get_logger().info("Reached target score — game finished")
        except Exception as e:
            self.get_logger().error(f"Error in qrcode_callback: {e}")

    def _detect_qr_texts(self, frame) -> List[str]:
        # detector removed: QR results come from `qrcode.py` topic
        return []

    def start_game(self):
        self.found = []
        self.found_set = set()
        self.start_time = time.time()
        self.state = "running"
        self.get_logger().info("Game started")

    def reset_game(self):
        self.found = []
        self.found_set = set()
        self.start_time = 0.0
        self.state = "waiting"
        self.get_logger().info("Game reset")

    def keyboard_pressed_callback(self, msg: Int32) -> None:
        try:
            key_val = int(msg.data)
            # start
            if key_val == ord('i'):
                if self.state != "running":
                    self.start_game()
            # reset
            elif key_val == ord('o'):
                self.reset_game()
            # quit
            elif key_val == ord('p'):
                self._should_exit = True
        except Exception as e:
            self.get_logger().error(f"Error in keyboard_pressed_callback: {e}")

    def update(self):
        # called from main loop to process frame and update game
        frame = self.latest_frame.copy() if self.latest_frame is not None else None

        if frame is None:
            # show a black screen with message
            blank = 255 * (np.ones((480, 640, 3), dtype="uint8"))
            cv2.putText(blank, "Waiting for camera...", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 0, 255), 2)
            cv2.imshow("QR Game", blank)
            return

        # check timeout while running
        if self.state == "running":
            elapsed = time.time() - self.start_time
            if elapsed >= self.game_time:
                self.state = "finished"

        # overlay UI
        try:
            overlay = frame.copy()
            h, w = overlay.shape[:2]
            line_height = 40
            y = line_height
            # header
            cv2.putText(overlay, f"State: {self.state}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 0), 3)
            if self.state == "running":
                remaining = max(0, int(self.game_time - (time.time() - self.start_time)))
            else:
                remaining = int(self.game_time)
            y += line_height
            cv2.putText(overlay, f"Score: {len(self.found)}/{self.score_max}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 0), 3)
            y += line_height
            cv2.putText(overlay, f"Time: {remaining}s", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 0), 3)

            # list found codes
            y += line_height
            for idx, txt in enumerate(self.found[-6:]):
                cv2.putText(overlay, f"{idx+1}. {txt}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (200, 200, 200), 1)
                y += line_height

            # hint
            cv2.putText(overlay, "Controls: i=start, o=reset, p=quit", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 1)

            cv2.imshow("QR Game", overlay)
        except Exception as exc:
            self.get_logger().error(f"Display error: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = QRGameNode()

    window_name = "QR Game"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    try:
        while rclpy.ok():
            # process callbacks
            rclpy.spin_once(node, timeout_sec=0)

            node.update()

            # Refresh OpenCV window (do not handle key input here)
            cv2.waitKey(30)

            # Exit if keyboard handler requested quit
            if getattr(node, '_should_exit', False):
                break

            # if finished, show final overlay but keep window until user resets or quits
        cv2.destroyAllWindows()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

