"""person_crop_node.py — 사람 인식 후 영역 크롭 퍼블리시 ROS2 노드.

ROS2 이미지 토픽을 구독하여 사람이 인식되면 해당 영역(바운딩 박스)을
별도 이미지 토픽으로 퍼블리시합니다.
사람이 여럿일 경우 가장 큰 영역 하나만 퍼블리시합니다.

검출 방식:
  OpenCV HOG (Histogram of Oriented Gradients) + LinearSVM
  — 추가 설치 없이 opencv-python만으로 동작

파라미터:
  image_topic    (str,   기본 "/image_raw")     — 구독할 입력 이미지 토픽
  person_topic   (str,   기본 "/person_image")  — 크롭 이미지 퍼블리시 토픽
  scale          (float, 기본 1.05)             — HOG 피라미드 스케일. 작을수록 정밀·느림
  win_stride     (int,   기본 8)                — 슬라이딩 윈도우 스트라이드 (px)
  padding        (float, 기본 0.1)              — bbox 주변 여백 비율 (0.1 = 10%)
  min_area       (int,   기본 3000)             — 검출 최소 면적 (px²). 너무 작은 박스 무시
  nms_threshold  (float, 기본 0.65)             — Non-Maximum Suppression overlap 임계값
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def _nms(boxes, scores, threshold: float):
    """간단한 NMS (Non-Maximum Suppression).

    boxes  : (N, 4) ndarray — (x, y, w, h)
    scores : (N,)   ndarray
    반환   : 살아남은 인덱스 리스트
    """
    if len(boxes) == 0:
        return []

    x1 = boxes[:, 0].astype(float)
    y1 = boxes[:, 1].astype(float)
    x2 = (boxes[:, 0] + boxes[:, 2]).astype(float)
    y2 = (boxes[:, 1] + boxes[:, 3]).astype(float)
    areas = (x2 - x1) * (y2 - y1)

    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
        order = order[1:][iou < threshold]

    return keep


class PersonCropNode(Node):
    """사람 검출 후 크롭 이미지 퍼블리시 노드."""

    def __init__(self):
        super().__init__("person_crop_node")

        # ── 파라미터 ──────────────────────────────────────────────────────
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("person_topic", "/person_image")
        self.declare_parameter("scale", 1.05)
        self.declare_parameter("win_stride", 8)
        self.declare_parameter("padding", 0.1)
        self.declare_parameter("min_area", 3000)
        self.declare_parameter("nms_threshold", 0.65)

        image_topic = self.get_parameter("image_topic").value
        person_topic = self.get_parameter("person_topic").value
        self._scale = self.get_parameter("scale").value
        stride = self.get_parameter("win_stride").value
        self._padding = self.get_parameter("padding").value
        self._min_area = self.get_parameter("min_area").value
        self._nms_thr = self.get_parameter("nms_threshold").value

        self._win_stride = (stride, stride)

        # ── HOG 검출기 ────────────────────────────────────────────────────
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # ── CvBridge ──────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── 퍼블리셔 / 구독자 ─────────────────────────────────────────────
        self._pub = self.create_publisher(Image, person_topic, 10)
        self.create_subscription(Image, image_topic, self._image_callback, 10)

        self.get_logger().info(
            f"PersonCropNode 준비 | 입력: {image_topic} → 출력: {person_topic}"
        )

    def _image_callback(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"CvBridge 변환 실패: {e}")
            return

        h, w = frame.shape[:2]

        # HOG 검출 시 고해상도 이미지는 처리 시간이 길어지므로 640px 너비로 축소
        scale_factor = 1.0
        if w > 640:
            scale_factor = 640.0 / w
            detect_frame = cv2.resize(frame, (640, int(h * scale_factor)))
        else:
            detect_frame = frame

        boxes, scores = self._hog.detectMultiScale(
            detect_frame,
            winStride=self._win_stride,
            scale=self._scale,
        )

        if len(boxes) == 0:
            return

        # 스케일 보정 (원본 해상도 좌표로)
        if scale_factor != 1.0:
            boxes = (boxes / scale_factor).astype(int)

        scores = np.array(scores).flatten()
        boxes = np.array(boxes)

        # 최소 면적 필터
        areas = boxes[:, 2] * boxes[:, 3]
        mask = areas >= self._min_area
        boxes = boxes[mask]
        scores = scores[mask]
        areas = areas[mask]

        if len(boxes) == 0:
            return

        # NMS
        keep = _nms(boxes, scores, self._nms_thr)
        boxes = boxes[keep]
        areas = areas[keep]

        # 가장 큰 박스 선택
        best = int(np.argmax(areas))
        x, y, bw, bh = boxes[best]

        # 패딩 적용
        pad_x = int(bw * self._padding)
        pad_y = int(bh * self._padding)
        x1 = max(0, x - pad_x)
        y1 = max(0, y - pad_y)
        x2 = min(w, x + bw + pad_x)
        y2 = min(h, y + bh + pad_y)

        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            return

        try:
            out_msg = self._bridge.cv2_to_imgmsg(crop, encoding="bgr8")
            out_msg.header = msg.header  # 타임스탬프·frame_id 유지
            self._pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f"퍼블리시 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PersonCropNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
