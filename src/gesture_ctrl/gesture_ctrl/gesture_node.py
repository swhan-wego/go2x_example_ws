#!/usr/bin/env python3
"""
gesture_node.py — ROS 토픽 기반 손 제스처 인식 ROS2 노드

/image_raw (sensor_msgs/Image) 토픽에서 이미지를 수신하여 손 제스처를 인식하고
build_cmd 토픽으로 로봇 명령을 전송하는 ROS2 노드입니다.

제스처 매핑:
  ✊ 주먹          → stop
  ☝️ 검지만         → forward
  ✌️ 검지+중지      → backward
  👍 엄지 왼쪽      → turn_left
  👉 엄지 오른쪽    → turn_right
  🖐 다섯 손가락    → emergency_stop
  🤙 약지+소지      → sit
  🖖 소지만         → stand_up
  🤘 중지+약지+소지 → lie_down

실행:
  ros2 run gesture_ctrl gesture_node
"""

import json
import os
import time

import cv2
import mediapipe as mp
import rclpy
from cv_bridge import CvBridge
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.core import base_options as base_options_module
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from gesture_ctrl.finger_utils import GestureSmoothing, classify_gesture

# 모델 파일 경로
_MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "hand_landmarker.task")

# 제스처 문자열 → build_cmd JSON 매핑
GESTURE_TO_CMD = {
    "forward":        {"type": "move", "x": 0.5,  "y": 0.0, "z": 0.0},
    "backward":       {"type": "move", "x": -0.5, "y": 0.0, "z": 0.0},
    "turn_left":      {"type": "move", "x": 0.0,  "y": 0.0, "z": 1.0},
    "turn_right":     {"type": "move", "x": 0.0,  "y": 0.0, "z": -1.0},
    "stop":           {"type": "move", "x": 0.0,  "y": 0.0, "z": 0.0},
    "emergency_stop": {"type": "damp"},
    "sit":            {"type": "sit"},
    "stand_up":       {"type": "stand_up"},
    "lie_down":       {"type": "stand_down"},
}

# 랜드마크 연결선 (시각화용)
_HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),       # 엄지
    (0, 5), (5, 6), (6, 7), (7, 8),       # 검지
    (0, 9), (9, 10), (10, 11), (11, 12),  # 중지
    (0, 13), (13, 14), (14, 15), (15, 16), # 약지
    (0, 17), (17, 18), (18, 19), (19, 20), # 소지
    (5, 9), (9, 13), (13, 17),            # 손바닥
]


def _draw_hand(frame, lm_list):
    """랜드마크 점과 연결선을 frame에 그립니다."""
    h, w = frame.shape[:2]
    pts = [(int(lm.x * w), int(lm.y * h)) for lm in lm_list]
    for a, b in _HAND_CONNECTIONS:
        cv2.line(frame, pts[a], pts[b], (0, 200, 0), 2)
    for x, y in pts:
        cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)


class GestureNode(Node):
    """
    ROS 이미지 토픽에서 손 제스처를 인식하여 build_cmd 토픽으로 로봇 명령을 전송하는 노드.

    파라미터:
        image_topic   (str, 기본 "/image_raw") — 구독할 이미지 토픽
        smooth_window (int, 기본 10)           — 제스처 스무딩 윈도우 크기
    """

    def __init__(self):
        super().__init__("gesture_node")

        # ── 파라미터 ──────────────────────────────────────────────────────
        self.declare_parameter("image_topic",   "/image_raw")
        self.declare_parameter("smooth_window", 10)

        image_topic = self.get_parameter("image_topic").value
        smooth_win  = self.get_parameter("smooth_window").value

        # ── MediaPipe HandLandmarker (Tasks API) ──────────────────────────
        options = vision.HandLandmarkerOptions(
            base_options=base_options_module.BaseOptions(
                model_asset_path=_MODEL_PATH
            ),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.7,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self._detector = vision.HandLandmarker.create_from_options(options)

        # ── CvBridge ──────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── 스무딩 ────────────────────────────────────────────────────────
        self._smoother = GestureSmoothing(window=smooth_win)

        # ── 상태 ──────────────────────────────────────────────────────────
        self._last_gesture = ""

        # ── ROS2 퍼블리셔 / 구독자 ────────────────────────────────────────
        self._pub = self.create_publisher(String, "build_cmd", 10)
        self.create_subscription(Image, image_topic, self._image_callback, 10)

        self.get_logger().info(f"GestureNode 준비 완료 | 구독: {image_topic}")

    def _image_callback(self, msg: Image):
        # sensor_msgs/Image → BGR numpy
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"CvBridge 변환 실패: {e}")
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 종료 키
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("종료 요청")
            rclpy.shutdown()
            return

        # ── 손 제스처 인식 ────────────────────────────────────────────────
        mp_image  = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        timestamp = int(time.monotonic() * 1000)
        result    = self._detector.detect_for_video(mp_image, timestamp)

        gesture = "unknown"
        if result.hand_landmarks and result.handedness:
            lm         = result.hand_landmarks[0]
            handedness = result.handedness[0][0].category_name  # "Left" / "Right"
            _draw_hand(frame, lm)
            gesture = classify_gesture(lm, handedness)

        smoothed = self._smoother.update(gesture)

        # 상태 변화가 있을 때만 퍼블리시
        skip = {"unknown", "idle"}
        if smoothed not in skip and smoothed != self._last_gesture:
            self._last_gesture = smoothed
            cmd = GESTURE_TO_CMD.get(smoothed)
            if cmd is not None:
                msg_out = String()
                msg_out.data = json.dumps(cmd)
                self._pub.publish(msg_out)
                self.get_logger().info(f"제스처 → {smoothed} | 명령: {cmd}")
            else:
                self.get_logger().warn(f"매핑 없는 제스처: {smoothed}")

        # ── HUD ───────────────────────────────────────────────────────────
        color = (0, 200, 0) if smoothed not in skip else (100, 100, 100)
        cv2.putText(
            frame, f"Gesture: {smoothed}",
            (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2
        )
        cv2.putText(
            frame, "q: quit",
            (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1
        )
        cv2.imshow("Gesture Control", frame)

    def destroy_node(self):
        self._detector.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
