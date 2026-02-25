"""웹캠 이미지 퍼블리시 ROS2 노드.

웹캠 화면을 입력 받아 ROS2 이미지 토픽으로 퍼블리시하는 노드입니다.
OpenCV의 VideoCapture를 사용하여 웹캠에서 프레임을 읽어들입니다.
퍼블리시 주기(프레임 레이트)는 파라미터로 설정할 수 있습니다.
퍼블리시되는 이미지 토픽은 sensor_msgs/Image 타입입니다.
센서의 밝기, 대비, 채도 등을 OpenCV로 조절할 수 있습니다.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class WebcamNode(Node):
    def __init__(self):
        super().__init__("webcam_node")

        # 파라미터 선언
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("frame_rate", 30)
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("brightness", 0)
        self.declare_parameter("contrast", 1.0)
        self.declare_parameter("saturation", 1.0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)

        # 파라미터 가져오기
        self.camera_id = self.get_parameter("camera_id").get_parameter_value().integer_value
        self.frame_rate = self.get_parameter("frame_rate").get_parameter_value().integer_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.brightness = self.get_parameter("brightness").get_parameter_value().integer_value
        self.contrast = self.get_parameter("contrast").get_parameter_value().double_value
        self.saturation = self.get_parameter("saturation").get_parameter_value().double_value
        self.frame_width = self.get_parameter("frame_width").get_parameter_value().integer_value
        self.frame_height = self.get_parameter("frame_height").get_parameter_value().integer_value

        # 웹캠 초기화
        self.cap = cv2.VideoCapture(self.camera_id)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera {self.camera_id}")
            raise RuntimeError(f"Cannot open camera {self.camera_id}")

        # 카메라 해상도 설정
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)

        # 퍼블리셔 및 CvBridge 초기화
        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        self.bridge = CvBridge()

        # 타이머 생성 (프레임 레이트에 맞춰 콜백 실행)
        self.timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            f"Webcam node started. camera_id={self.camera_id}, frame_rate={self.frame_rate}fps, "
            f"brightness={self.brightness}, contrast={self.contrast:.1f}, saturation={self.saturation:.1f}"
        )

    def adjust_image(self, frame):
        """이미지 밝기, 대비, 채도 조정"""
        # 밝기 조정
        if self.brightness != 0:
            frame = cv2.convertScaleAbs(frame, alpha=1, beta=self.brightness)

        # 대비 조정
        if self.contrast != 1.0:
            frame = cv2.convertScaleAbs(frame, alpha=self.contrast, beta=0)

        # 채도 조정
        if self.saturation != 1.0:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[:, :, 1] = hsv[:, :, 1] * self.saturation
            hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)
            frame = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

        return frame

    def timer_callback(self):
        """타이머 콜백: 프레임 캡처 및 퍼블리시"""
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Failed to read frame from camera")
            return

        # 이미지 조정
        frame = self.adjust_image(frame)

        # ROS2 Image 메시지로 변환 및 퍼블리시
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")

    def destroy_node(self):
        """노드 종료 시 웹캠 해제"""
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

