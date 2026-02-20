"""QR CODE 인식 ROS2 노드.

이미지를 입력 받고 QR CODE를 인식하여 결과를 반환합니다.
일회용(once)으로 인식할지 반복 인식할지 파라미터로 선택 가능합니다.
인식된 QR CODE 데이터는 문자열로 발행됩니다.
cv2.wechat_qrcode_WeChatQRCode 클래스를 사용합니다.
"""

import os
from typing import Optional

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge


class QRCodeNode(Node):
	def __init__(self):
		super().__init__("qrcode_node")

		# 현재 파일(qrcode.py)의 절대 경로를 기준으로 기본 모델 디렉토리 설정
		# go2_libs/qrcode.py 위치에서 상위 폴더인 workspace root의 models 폴더를 가리킴
		current_file_path = os.path.abspath(__file__)
		project_root = os.path.dirname(os.path.dirname(current_file_path))
		default_model_dir = os.path.join(project_root, "models")

		self.declare_parameter("image_topic", "/image_raw")
		self.declare_parameter("repeat_same", False)
		self.declare_parameter("cooldown", 5.0)
		self.declare_parameter("model_dir", default_model_dir)
  
		self.declare_parameter("detect_prototxt", "detect_2021nov.prototxt")
		self.declare_parameter("detect_caffemodel", "detect_2021nov.caffemodel")
		self.declare_parameter("sr_prototxt", "sr_2021nov.prototxt")
		self.declare_parameter("sr_caffemodel", "sr_2021nov.caffemodel")
  
		self.declare_parameter("output_topic", "/qrcode")

		self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
		self.repeat_same = self.get_parameter("repeat_same").get_parameter_value().bool_value
		self.cooldown = self.get_parameter("cooldown").get_parameter_value().double_value
		model_dir = self.get_parameter("model_dir").get_parameter_value().string_value
		detect_prototxt = self.get_parameter("detect_prototxt").get_parameter_value().string_value
		detect_caffemodel = self.get_parameter("detect_caffemodel").get_parameter_value().string_value
		sr_prototxt = self.get_parameter("sr_prototxt").get_parameter_value().string_value
		sr_caffemodel = self.get_parameter("sr_caffemodel").get_parameter_value().string_value
		output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

		detect_prototxt_path = os.path.join(model_dir, detect_prototxt)
		detect_caffemodel_path = os.path.join(model_dir, detect_caffemodel)
		sr_prototxt_path = os.path.join(model_dir, sr_prototxt)
		sr_caffemodel_path = os.path.join(model_dir, sr_caffemodel)

		# 모델 파일 존재 여부 확인
		model_paths = [detect_prototxt_path, detect_caffemodel_path, sr_prototxt_path, sr_caffemodel_path]
		missing_files = [p for p in model_paths if not os.path.exists(p)]

		if missing_files:
			self.get_logger().error("QR 모델 파일이 누락되었습니다!")
			if missing_files:
				for f in missing_files:
					self.get_logger().error(f"누락된 파일: {f}")
				self.get_logger().info(
					"모델 다운로드: https://huggingface.co/opencv/qrcode_wechatqrcode/tree/main"
				)
				raise RuntimeError("QR 모델 파일 누락으로 노드를 초기화할 수 없습니다.")
				# 모델이 없으면 노드 실행이 불가능하므로 예외 발생 또는 초기화 중단
		
		self.detector = cv2.wechat_qrcode_WeChatQRCode(
			detect_prototxt_path,
			detect_caffemodel_path,
			sr_prototxt_path,
			sr_caffemodel_path,
		)

		self.bridge = CvBridge()
		self.publisher = self.create_publisher(String, output_topic, 10)
		self.subscription = self.create_subscription(
			Image, self.image_topic, self.image_callback, 10
		)

		# Service to clear last_detected dictionary
		self.clear_service = self.create_service(
			Trigger, "~/clear_last_detected", self.clear_last_detected_callback
		)
		self.get_logger().info(
			f"Clear service available at: "
			f"/{self.get_name()}/clear_last_detected"
)

		self.last_detected = {} # {qr_text: last_seen_time}
		self.get_logger().info(
			f"QRCode node started. image_topic={self.image_topic}, repeat_same={self.repeat_same}, cooldown={self.cooldown}s"
		)

	def clear_last_detected_callback(self, request, response):
		"""Clear the last_detected dictionary."""
		count = len(self.last_detected)
		self.last_detected.clear()
		response.success = True
		response.message = f"Cleared {count} detected QR code(s)"
		self.get_logger().info(response.message)
		return response

	def _publish_result(self, text: str) -> None:
		msg = String()
		msg.data = text
		self.publisher.publish(msg)
		self.get_logger().info(f"QR detected: {text}")

	def _detect_qr(self, frame) -> Optional[str]:
		try:
			texts, _points = self.detector.detectAndDecode(frame)
			if texts:
				return texts[0]
		except Exception as exc:
			self.get_logger().error(f"QR detection failed: {exc}")
		return None

	def image_callback(self, msg: Image) -> None:
		try:
			frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
		except Exception as e:
			self.get_logger().error(f"이미지 변환 실패: {e}")
			return
		result = self._detect_qr(frame)
		
		if result:
			current_time = self.get_clock().now().nanoseconds / 1e9
			last_time = self.last_detected.get(result, 0.0)

			if not self.repeat_same:
				# 동일 코드 재인식 안 함: 처음 발견했을 때만 발행
				if result not in self.last_detected:
					self._publish_result(result)
					self.last_detected[result] = current_time
			else:
				# 주기적으로 인식: 쿨타임이 지났으면 다시 발행
				if current_time - last_time >= self.cooldown:
					self._publish_result(result)
					self.last_detected[result] = current_time


def main(args=None):
	rclpy.init(args=args)
	node = QRCodeNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()