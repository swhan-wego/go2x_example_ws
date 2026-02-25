"""색깔별 블록 추적 ROS2 노드.

이미지를 입력 받고 HSV 색공간에서 색깔별 블록을 검출하여 결과를 발행합니다.
검출된 블록의 색상, 위치(중심점), 크기(면적)를 JSON 문자열로 발행합니다.
추적할 색상과 HSV 범위는 파라미터로 설정 가능합니다.
"""

import json
import os
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge


# 기본 색상별 HSV 범위 정의
# HSV: Hue(0~179), Saturation(0~255), Value(0~255)
DEFAULT_COLOR_RANGES = {
    "red": [
        {"lower": [0, 100, 100],   "upper": [10, 255, 255]},   # 빨강 (0도 근처)
        {"lower": [170, 100, 100], "upper": [179, 255, 255]},  # 빨강 (180도 근처, 랩어라운드)
    ],
    "green":  [{"lower": [40, 100, 100],  "upper": [80, 255, 255]}],
    "blue":   [{"lower": [100, 100, 100], "upper": [130, 255, 255]}],
    "yellow": [{"lower": [20, 100, 100],  "upper": [40, 255, 255]}],
}


class ColorBlockTrackerNode(Node):
    def __init__(self):
        super().__init__("color_block_tracker_node")

        # ── 파라미터 선언 ────────────────────────────────────────────
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("output_topic", "/color_blocks")
        self.declare_parameter("min_area", 500.0)        # 검출 최소 면적 (px²)
        self.declare_parameter("max_area", 100000.0)     # 검출 최대 면적 (px²)
        self.declare_parameter("blur_kernel", 5)         # 가우시안 블러 커널 크기
        self.declare_parameter("draw_debug", True)       # 디버그 이미지 발행 여부
        self.declare_parameter("debug_topic", "/color_blocks/debug")

        self.image_topic  = self.get_parameter("image_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.min_area     = self.get_parameter("min_area").get_parameter_value().double_value
        self.max_area     = self.get_parameter("max_area").get_parameter_value().double_value
        self.blur_kernel  = self.get_parameter("blur_kernel").get_parameter_value().integer_value
        self.draw_debug   = self.get_parameter("draw_debug").get_parameter_value().bool_value
        self.debug_topic  = self.get_parameter("debug_topic").get_parameter_value().string_value

        # blur_kernel은 반드시 홀수여야 함
        if self.blur_kernel % 2 == 0:
            self.blur_kernel += 1
            self.get_logger().warn(
                f"blur_kernel은 홀수여야 합니다. {self.blur_kernel - 1} → {self.blur_kernel}으로 조정됨"
            )

        # ── 색상 범위 초기화 ─────────────────────────────────────────
        self.color_ranges = DEFAULT_COLOR_RANGES

        # ── QoS 설정 ─────────────────────────────────────────────────
        # 이미지 구독: 실시간성 우선 (BEST_EFFORT, 최신 1프레임만 유지)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # ── 구독 / 발행 ──────────────────────────────────────────────
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, image_qos
        )
        self.result_pub = self.create_publisher(String, self.output_topic, 10)

        if self.draw_debug:
            self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)

        # ── 서비스 ───────────────────────────────────────────────────
        self.clear_service = self.create_service(
            Trigger, "~/reset_color_ranges", self.reset_color_ranges_callback
        )

        self.get_logger().info(
            f"ColorBlockTracker started.\n"
            f"  image_topic : {self.image_topic}\n"
            f"  output_topic: {self.output_topic}\n"
            f"  min_area    : {self.min_area} px²\n"
            f"  colors      : {list(self.color_ranges.keys())}\n"
            f"  debug       : {self.draw_debug} → {self.debug_topic}"
        )
        self.get_logger().info(
            f"Reset service: /{self.get_name()}/reset_color_ranges"
        )

    # ── 서비스 콜백 ──────────────────────────────────────────────────
    def reset_color_ranges_callback(self, request, response):
        self.color_ranges = DEFAULT_COLOR_RANGES
        response.success = True
        response.message = f"색상 범위가 기본값으로 초기화되었습니다: {list(self.color_ranges.keys())}"
        self.get_logger().info(response.message)
        return response

    # ── 이미지 콜백 ──────────────────────────────────────────────────
    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
            return

        detections, debug_frame = self._detect_color_blocks(frame)

        # 결과 발행
        result_msg = String()
        result_msg.data = json.dumps(detections, ensure_ascii=False)
        self.result_pub.publish(result_msg)

        if detections:
            self.get_logger().info(f"검출된 블록: {[d['color'] for d in detections]}")

        # 디버그 이미지 발행
        if self.draw_debug and debug_frame is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
                debug_msg.header = msg.header   # 원본 타임스탬프 유지
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"디버그 이미지 발행 실패: {e}")

    # ── 핵심 검출 로직 ───────────────────────────────────────────────
    def _detect_color_blocks(self, frame: np.ndarray) -> tuple[list, Optional[np.ndarray]]:
        detections = []
        debug_frame = frame.copy() if self.draw_debug else None

        # 전처리: 가우시안 블러로 노이즈 제거
        blurred = cv2.GaussianBlur(
            frame,
            (self.blur_kernel, self.blur_kernel),
            0
        )
        # BGR → HSV 변환
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        for color_name, ranges in self.color_ranges.items():
            mask = self._build_mask(hsv, ranges)
            blocks = self._find_blocks(mask, color_name)

            for block in blocks:
                detections.append(block)

                if self.draw_debug and debug_frame is not None:
                    self._draw_block(debug_frame, block, color_name)

        return detections, debug_frame

    def _build_mask(self, hsv: np.ndarray, ranges: list) -> np.ndarray:
        """복수의 HSV 범위를 OR 합성하여 최종 마스크 생성."""
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for r in ranges:
            lower = np.array(r["lower"], dtype=np.uint8)
            upper = np.array(r["upper"], dtype=np.uint8)
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

        # 모폴로지 연산으로 마스크 노이즈 제거 및 홀 채우기
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)  # 작은 노이즈 제거
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 내부 홀 채우기
        return mask

    def _find_blocks(self, mask: np.ndarray, color_name: str) -> list:
        """마스크에서 윤곽선을 찾아 블록 정보 리스트로 반환."""
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        blocks = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (self.min_area <= area <= self.max_area):
                continue

            # 바운딩 박스 및 중심점 계산
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            cy = y + h // 2

            # 모멘트 기반 정밀 중심점
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

            blocks.append({
                "color":    color_name,
                "center_x": cx,
                "center_y": cy,
                "area":     float(area),
                "bbox":     {"x": x, "y": y, "w": w, "h": h},
            })

        # 면적 기준 내림차순 정렬 (큰 블록 우선)
        blocks.sort(key=lambda b: b["area"], reverse=True)
        return blocks

    def _draw_block(self, frame: np.ndarray, block: dict, color_name: str) -> None:
        """디버그 이미지에 바운딩 박스, 중심점, 레이블 시각화."""
        COLOR_BGR = {
            "red":    (0, 0, 255),
            "green":  (0, 255, 0),
            "blue":   (255, 0, 0),
            "yellow": (0, 255, 255),
        }
        bgr = COLOR_BGR.get(color_name, (255, 255, 255))
        b = block["bbox"]

        cv2.rectangle(frame, (b["x"], b["y"]), (b["x"] + b["w"], b["y"] + b["h"]), bgr, 2)
        cv2.circle(frame, (block["center_x"], block["center_y"]), 5, bgr, -1)
        cv2.putText(
            frame,
            f"{color_name} ({block['area']:.0f}px)",
            (b["x"], b["y"] - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, bgr, 2
        )


def main(args=None):
    rclpy.init(args=args)
    node = ColorBlockTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()