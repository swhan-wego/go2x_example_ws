"""object_yolo_node.py — YOLOv8 ONNX 기반 사물 인식 후 크롭 퍼블리시 ROS2 노드.

onnxruntime을 사용하여 YOLOv8n ONNX 모델로 지정한 사물을 인식하고,
검출된 영역을 ROS2 이미지 토픽으로 퍼블리시합니다.
torch/ultralytics 없이 동작합니다.

모델 준비 (최초 1회):
  cd <workspace>
  python3 src/image_agent/scripts/download_yolo_model.py
  # → models/yolov8n.onnx 생성

파라미터:
  image_topic    (str,   기본 "/image_raw")              — 입력 이미지 토픽
  object_topic   (str,   기본 "/object_image")           — 크롭 이미지 출력 토픽
  model_path     (str,   기본 "models/yolov8n.onnx")    — ONNX 모델 파일 경로
                         절대경로 또는 워크스페이스 상대경로
  target_class   (int,   기본 0)    — 추적할 COCO 클래스 ID
                         주요 COCO 클래스:
                           0=person, 1=bicycle, 2=car, 3=motorcycle,
                           5=bus, 7=truck, 14=bird, 15=cat, 16=dog,
                           39=bottle, 56=chair, 57=couch, 63=laptop,
                           67=cell phone, 73=book
  conf_threshold (float, 기본 0.4)  — 최소 신뢰도
  iou_threshold  (float, 기본 0.45) — NMS IoU 임계값
  input_size     (int,   기본 640)  — 모델 입력 크기 (정사각형 한 변 px)
  padding        (float, 기본 0.05) — bbox 주변 여백 비율
  publish_all    (bool,  기본 False) — True: 검출된 모든 사물 각각 발행
                                       False: 가장 큰 사물 1개만 발행
  object_pos_topic (str, 기본 "/object_position") — 위치 출력 토픽 (PointStamped)
                         x, y: 정규화된 중심 좌표 (0.0~1.0)
                         z: 검출 신뢰도 (confidence)
                         * publish_all=False 일 때 가장 높은 신뢰도 사물 1개만 발행
"""

import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import onnxruntime as ort
except ImportError:
    ort = None


def _letterbox(img: np.ndarray, size: int):
    """이미지를 정사각형(size×size)으로 letterbox 변환.

    반환: (변환된 이미지, 스케일, (pad_w, pad_h))
    """
    h, w = img.shape[:2]
    scale = size / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    pad_w = (size - new_w) // 2
    pad_h = (size - new_h) // 2
    padded = cv2.copyMakeBorder(
        resized,
        pad_h, size - new_h - pad_h,
        pad_w, size - new_w - pad_w,
        cv2.BORDER_CONSTANT, value=(114, 114, 114),
    )
    return padded, scale, (pad_w, pad_h)


def _decode_boxes(output: np.ndarray, scale: float, pad: tuple, orig_hw: tuple,
                  conf_thr: float, target_class: int):
    """YOLOv8 출력 텐서 디코딩.

    output: (84, 8400) — [cx, cy, w, h, cls0_score, ..., cls79_score]
    반환: (boxes_xyxy, scores) — target_class 클래스만
    """
    # output shape: (84, 8400) → transpose → (8400, 84)
    preds = output.T  # (8400, 84)

    # target_class 점수
    scores = preds[:, 4 + target_class]
    mask = scores >= conf_thr
    if not mask.any():
        return np.empty((0, 4)), np.empty(0)

    preds  = preds[mask]
    scores = scores[mask]

    # cx, cy, w, h → 원본 이미지 좌표계
    cx = (preds[:, 0] - pad[0]) / scale
    cy = (preds[:, 1] - pad[1]) / scale
    bw = preds[:, 2] / scale
    bh = preds[:, 3] / scale

    oh, ow = orig_hw
    x1 = np.clip(cx - bw / 2, 0, ow)
    y1 = np.clip(cy - bh / 2, 0, oh)
    x2 = np.clip(cx + bw / 2, 0, ow)
    y2 = np.clip(cy + bh / 2, 0, oh)

    boxes = np.stack([x1, y1, x2, y2], axis=1)
    return boxes, scores


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_thr: float):
    """OpenCV NMS 래퍼 (x1y1x2y2 입력)."""
    if len(boxes) == 0:
        return []
    xywh = boxes.copy()
    xywh[:, 2] -= xywh[:, 0]  # w
    xywh[:, 3] -= xywh[:, 1]  # h
    indices = cv2.dnn.NMSBoxes(
        xywh.tolist(), scores.tolist(), score_threshold=0.0, nms_threshold=iou_thr
    )
    if len(indices) == 0:
        return []
    return indices.flatten().tolist()


class ObjectYoloNode(Node):
    """YOLOv8n ONNX 사물 검출 후 크롭 이미지 퍼블리시 노드."""

    def __init__(self):
        super().__init__("object_yolo_node")

        if ort is None:
            self.get_logger().fatal("onnxruntime 설치 필요: pip install onnxruntime")
            raise RuntimeError("onnxruntime not found")

        # ── 파라미터 ──────────────────────────────────────────────────────
        self.declare_parameter("image_topic",      "/image_raw")
        self.declare_parameter("object_topic",     "/object_image")
        self.declare_parameter("object_pos_topic", "/object_position")
        self.declare_parameter("model_path",       "models/yolov8n.onnx")
        self.declare_parameter("target_class",     0)
        self.declare_parameter("conf_threshold",   0.4)
        self.declare_parameter("iou_threshold",    0.45)
        self.declare_parameter("input_size",       640)
        self.declare_parameter("padding",          0.05)
        self.declare_parameter("publish_all",      False)

        image_topic      = self.get_parameter("image_topic").value
        object_topic     = self.get_parameter("object_topic").value
        object_pos_topic = self.get_parameter("object_pos_topic").value
        model_path       = self.get_parameter("model_path").value
        self._target     = self.get_parameter("target_class").value
        self._conf       = self.get_parameter("conf_threshold").value
        self._iou        = self.get_parameter("iou_threshold").value
        self._sz         = self.get_parameter("input_size").value
        self._pad        = self.get_parameter("padding").value
        self._pub_all    = self.get_parameter("publish_all").value

        # 상대경로 → 워크스페이스 기준 절대경로 변환
        if not os.path.isabs(model_path):
            prefix = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)[0]
            ws_root = os.path.dirname(os.path.dirname(prefix)) if prefix else os.getcwd()
            model_path = os.path.join(ws_root, model_path)

        if not os.path.isfile(model_path):
            self.get_logger().fatal(
                f"모델 파일을 찾을 수 없습니다: {model_path}\n"
                "  python3 src/image_agent/scripts/download_yolo_model.py 로 다운로드하세요."
            )
            raise FileNotFoundError(f"Model not found: {model_path}")

        # ── ONNX 세션 ─────────────────────────────────────────────────────
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        self._session = ort.InferenceSession(model_path, providers=providers)
        self._input_name  = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name

        used_provider = self._session.get_providers()[0]
        self.get_logger().info(
            f"ONNX 모델 로드: {model_path} | provider: {used_provider}"
        )

        # ── CvBridge ──────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── 퍼블리셔 / 구독자 ─────────────────────────────────────────────
        self._pub     = self.create_publisher(Image, object_topic, 10)
        self._pub_pos = self.create_publisher(PointStamped, object_pos_topic, 10)
        self.create_subscription(Image, image_topic, self._image_callback, 10)

        self.get_logger().info(
            f"ObjectYoloNode 준비 | {image_topic} → {object_topic}, {object_pos_topic}"
            f" | target_class={self._target} conf={self._conf} iou={self._iou}"
        )

    def _preprocess(self, bgr: np.ndarray):
        """BGR numpy → ONNX 입력 텐서 (1, 3, sz, sz) float32."""
        lb, scale, pad = _letterbox(bgr, self._sz)
        rgb = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB)
        tensor = rgb.astype(np.float32) / 255.0
        tensor = np.transpose(tensor, (2, 0, 1))[np.newaxis]  # (1, 3, H, W)
        return tensor, scale, pad

    def _image_callback(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"CvBridge 변환 실패: {e}")
            return

        h, w = frame.shape[:2]

        # ── 추론 ──────────────────────────────────────────────────────────
        tensor, scale, pad = self._preprocess(frame)
        outputs = self._session.run(
            [self._output_name], {self._input_name: tensor}
        )
        # output: (1, 84, 8400) → squeeze → (84, 8400)
        output = outputs[0][0]

        boxes, scores = _decode_boxes(
            output, scale, pad, (h, w), self._conf, self._target
        )
        if len(boxes) == 0:
            return

        keep = _nms(boxes, scores, self._iou)
        boxes  = boxes[keep]
        scores = scores[keep]

        # 신뢰도 가장 높은 사물의 위치 발행 (publish_all 여부와 무관)
        best_score_idx = int(np.argmax(scores))
        self._publish_position(frame.shape, boxes[best_score_idx], scores[best_score_idx], msg.header)

        if not self._pub_all:
            # 가장 큰 bbox 하나만
            areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
            best  = int(np.argmax(areas))
            self._publish_crop(frame, boxes[best], msg.header)
        else:
            for box in boxes:
                self._publish_crop(frame, box, msg.header)

    def _publish_position(self, shape, box: np.ndarray, score: float, header):
        h, w = shape[:2]
        x1, y1, x2, y2 = box
        pt = PointStamped()
        pt.header = header
        pt.point.x = float((x1 + x2) / 2 / w)   # 정규화된 중심 x (0.0~1.0)
        pt.point.y = float((y1 + y2) / 2 / h)   # 정규화된 중심 y (0.0~1.0)
        pt.point.z = float(score)                  # 신뢰도
        self._pub_pos.publish(pt)

    def _publish_crop(self, frame: np.ndarray, box: np.ndarray, header):
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = box

        bw = x2 - x1
        bh = y2 - y1
        pad_x = int(bw * self._pad)
        pad_y = int(bh * self._pad)

        cx1 = max(0, int(x1) - pad_x)
        cy1 = max(0, int(y1) - pad_y)
        cx2 = min(w, int(x2) + pad_x)
        cy2 = min(h, int(y2) + pad_y)

        crop = frame[cy1:cy2, cx1:cx2]
        if crop.size == 0:
            return

        try:
            out_msg = self._bridge.cv2_to_imgmsg(crop, encoding="bgr8")
            out_msg.header = header
            self._pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f"퍼블리시 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
