"""색깔별 블록 시각화 ROS2 노드.

/color_blocks 토픽의 JSON 결과를 구독하여
이미지 위에 바운딩 박스, 중심점, 레이블을 그려 OpenCV 창으로 표시합니다.
"""

import json
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


# 색상별 BGR 값
COLOR_BGR = {
    "red":    (0, 0, 255),
    "green":  (0, 255, 0),
    "blue":   (255, 0, 0),
    "yellow": (0, 255, 255),
}


class ColorBlockVisualizerNode(Node):
    def __init__(self):
        super().__init__("color_block_visualizer_node")

        # ── 파라미터 선언 ────────────────────────────────────────────
        self.declare_parameter("image_topic",    "/image_raw")
        self.declare_parameter("blocks_topic",   "/color_blocks")
        self.declare_parameter("window_name",    "Color Block Tracker")
        self.declare_parameter("font_scale",     0.6)
        self.declare_parameter("line_thickness", 2)
        self.declare_parameter("show_area",      True)
        self.declare_parameter("show_center",    True)
        self.declare_parameter("show_crosshair", True)
        self.declare_parameter("window_width",   0)    # 0: 원본 크기 유지
        self.declare_parameter("window_height",  0)

        self.image_topic    = self.get_parameter("image_topic").get_parameter_value().string_value
        self.blocks_topic   = self.get_parameter("blocks_topic").get_parameter_value().string_value
        self.window_name    = self.get_parameter("window_name").get_parameter_value().string_value
        self.font_scale     = self.get_parameter("font_scale").get_parameter_value().double_value
        self.line_thickness = self.get_parameter("line_thickness").get_parameter_value().integer_value
        self.show_area      = self.get_parameter("show_area").get_parameter_value().bool_value
        self.show_center    = self.get_parameter("show_center").get_parameter_value().bool_value
        self.show_crosshair = self.get_parameter("show_crosshair").get_parameter_value().bool_value
        self.window_width   = self.get_parameter("window_width").get_parameter_value().integer_value
        self.window_height  = self.get_parameter("window_height").get_parameter_value().integer_value

        # ── 내부 상태 ────────────────────────────────────────────────
        self.bridge             = CvBridge()
        self.latest_detections: list = []

        # ── OpenCV 창 초기화 ─────────────────────────────────────────
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        if self.window_width > 0 and self.window_height > 0:
            cv2.resizeWindow(self.window_name, self.window_width, self.window_height)

        # ── QoS 설정 ─────────────────────────────────────────────────
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # ── 구독 ─────────────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, image_qos
        )
        self.blocks_sub = self.create_subscription(
            String, self.blocks_topic, self.blocks_callback, 10
        )

        # ── imshow를 위한 타이머 (33ms ≈ 30fps) ─────────────────────
        # cv2.imshow()는 반드시 메인 Thread에서 호출해야 하므로
        # rclpy 타이머 콜백에서 호출
        self.display_timer = self.create_timer(0.033, self.display_callback)
        self._pending_frame: Optional[np.ndarray] = None

        self.get_logger().info(
            f"ColorBlockVisualizer started.\n"
            f"  image_topic : {self.image_topic}\n"
            f"  blocks_topic: {self.blocks_topic}\n"
            f"  window      : '{self.window_name}'\n"
            f"  Q 또는 창 닫기로 종료"
        )

    # ── 검출 결과 콜백 ───────────────────────────────────────────────
    def blocks_callback(self, msg: String) -> None:
        try:
            self.latest_detections = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON 파싱 실패: {e}")
            self.latest_detections = []

    # ── 이미지 콜백 ──────────────────────────────────────────────────
    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
            return

        # 시각화 결과를 _pending_frame에 저장
        # 실제 imshow는 display_callback에서 수행
        self._pending_frame = self._draw_detections(frame, self.latest_detections)

    # ── 디스플레이 타이머 콜백 ───────────────────────────────────────
    def display_callback(self) -> None:
        if self._pending_frame is None:
            return

        cv2.imshow(self.window_name, self._pending_frame)
        key = cv2.waitKey(1) & 0xFF

    # ── 시각화 로직 ──────────────────────────────────────────────────
    def _draw_detections(self, frame: np.ndarray, detections: list) -> np.ndarray:
        vis = frame.copy()
        h, w = vis.shape[:2]

        if not detections:
            cv2.putText(
                vis, "No blocks detected",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                self.font_scale, (180, 180, 180), self.line_thickness
            )
            return vis

        for block in detections:
            color_name = block.get("color", "unknown")
            cx   = block.get("center_x", 0)
            cy   = block.get("center_y", 0)
            area = block.get("area", 0.0)
            bbox = block.get("bbox", {})
            bgr  = COLOR_BGR.get(color_name, (255, 255, 255))

            # 바운딩 박스
            if bbox:
                x, y, bw, bh = bbox["x"], bbox["y"], bbox["w"], bbox["h"]
                cv2.rectangle(vis, (x, y), (x + bw, y + bh), bgr, self.line_thickness)

                # 레이블 배경
                label = color_name
                if self.show_area:
                    label += f"  {area:.0f}px²"
                (tw, th), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX,
                    self.font_scale, self.line_thickness
                )
                cv2.rectangle(vis, (x, y - th - 10), (x + tw + 6, y), bgr, -1)
                cv2.putText(
                    vis, label, (x + 3, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    self.font_scale, (255, 255, 255), self.line_thickness
                )

            # 중심점
            if self.show_center:
                cv2.circle(vis, (cx, cy), 5, bgr, -1)
                cv2.circle(vis, (cx, cy), 8, (255, 255, 255), 1)

            # 크로스헤어 (화면 중심 → 블록 중심 연결선)
            if self.show_crosshair:
                screen_cx, screen_cy = w // 2, h // 2
                cv2.line(vis, (screen_cx, screen_cy), (cx, cy), bgr, 1, cv2.LINE_AA)
                cv2.drawMarker(
                    vis, (screen_cx, screen_cy),
                    (200, 200, 200), cv2.MARKER_CROSS, 20, 1
                )

        self._draw_legend(vis, detections)
        return vis

    def _draw_legend(self, frame: np.ndarray, detections: list) -> None:
        h, w = frame.shape[:2]
        counts: dict = {}
        for block in detections:
            c = block.get("color", "unknown")
            counts[c] = counts.get(c, 0) + 1

        padding  = 10
        line_h   = 22
        box_size = 14
        legend_w = 160
        legend_h = len(counts) * line_h + padding * 2

        overlay = frame.copy()
        lx = w - legend_w - padding
        ly = padding
        cv2.rectangle(overlay, (lx, ly), (lx + legend_w, ly + legend_h), (30, 30, 30), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

        for i, (color_name, count) in enumerate(counts.items()):
            bgr = COLOR_BGR.get(color_name, (255, 255, 255))
            y   = ly + padding + i * line_h
            cv2.rectangle(frame, (lx + 6, y), (lx + 6 + box_size, y + box_size), bgr, -1)
            cv2.putText(
                frame, f"{color_name}: {count}",
                (lx + 6 + box_size + 6, y + box_size - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1
            )


def main(args=None):
    rclpy.init(args=args)
    node = ColorBlockVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()