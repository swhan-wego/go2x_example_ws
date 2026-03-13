"""RTSP 스트림 이미지 퍼블리시 ROS2 노드.

RTSP 스트림에서 영상을 받아 ROS2 이미지 토픽으로 퍼블리시하는 노드입니다.
FFmpeg 또는 GStreamer 백엔드를 파라미터로 선택할 수 있습니다.
프레임이 도착하는 즉시 발행합니다 (타이머 폴링 없음).

백엔드:
  ffmpeg    : cv2.VideoCapture + CAP_FFMPEG (기본). 항상 동작.
  gstreamer : PyGObject(gi) GStreamer 직접 구동. cv2 불필요.
              software / vaapi / vaapi_legacy / nvdec 디코더 선택 가능.

파라미터:
  backend          : "ffmpeg" (기본) | "gstreamer"
  decoder          : "software" (기본) | "vaapi" | "vaapi_legacy" | "nvdec"
  gst_pipeline     : 직접 GStreamer 파이프라인 문자열 (비어있으면 자동 생성)
  gst_latency      : rtspsrc latency ms (기본 200)
  rtsp_url         : RTSP 스트림 URL
  image_topic      : 퍼블리시 토픽 (기본 /image_raw)
  frame_width      : 캡처 해상도 너비 (0 = 원본)  ← ffmpeg 전용
  frame_height     : 캡처 해상도 높이 (0 = 원본)  ← ffmpeg 전용
  reconnect_delay  : 재연결 대기 시간 초 (기본 2.0)
  stale_timeout    : 프레임 무응답 재연결 기준 초 (기본 5.0)
  status_interval  : 상태 로그 출력 주기 초 (기본 10.0)
"""

import threading
import time
from typing import Callable

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# GStreamer 디코더 × 코덱별 파이프라인 템플릿
# codec: "h264" | "h265"
# decoder: "software" | "vaapi" | "vaapi_legacy" | "nvdec"
_GST_PIPELINES = {
    ("software", "h264"): (
        "rtspsrc location={url} latency={latency} ! "
        "rtph264depay ! h264parse ! avdec_h264 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=appsink sync=false drop=true max-buffers=1"
    ),
    ("software", "h265"): (
        "rtspsrc location={url} latency={latency} ! "
        "rtph265depay ! h265parse ! avdec_h265 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=appsink sync=false drop=true max-buffers=1"
    ),
    ("vaapi", "h264"): (
        "rtspsrc location={url} latency={latency} ! "
        "rtph264depay ! h264parse ! vah264dec ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=appsink sync=false drop=true max-buffers=1"
    ),
    ("vaapi", "h265"): (
        "rtspsrc location={url} latency={latency} ! "
        "rtph265depay ! h265parse ! vah265dec ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=appsink sync=false drop=true max-buffers=1"
    ),
    ("vaapi_legacy", "h264"): (
        "rtspsrc location={url} latency={latency} ! "
        "rtph264depay ! h264parse ! vaapidecodebin ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=appsink sync=false drop=true max-buffers=1"
    ),
    ("vaapi_legacy", "h265"): (
        "rtspsrc location={url} latency={latency} ! "
        "rtph265depay ! h265parse ! vaapidecodebin ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=appsink sync=false drop=true max-buffers=1"
    ),
    ("nvdec", "h264"): (
        "rtspsrc location={url} latency={latency} ! "
        "rtph264depay ! h264parse ! nvh264dec ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=appsink sync=false drop=true max-buffers=1"
    ),
    ("nvdec", "h265"): (
        "rtspsrc location={url} latency={latency} ! "
        "rtph265depay ! h265parse ! nvh265dec ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=appsink sync=false drop=true max-buffers=1"
    ),
}
_DECODERS = {"software", "vaapi", "vaapi_legacy", "nvdec"}
_CODECS = {"h264", "h265"}


def _sample_to_bgr(sample) -> np.ndarray | None:
    buf = sample.get_buffer()
    caps = sample.get_caps()
    s = caps.get_structure(0)
    h = s.get_value("height")
    w = s.get_value("width")
    ok, info = buf.map(0)  # GST_MAP_READ = 0
    if not ok:
        return None
    frame = np.frombuffer(info.data, dtype=np.uint8).reshape((h, w, 3)).copy()
    buf.unmap(info)
    return frame


class _GstCapture:
    """GStreamer 기반 캡처. 프레임 도착 시 on_frame 콜백 호출."""

    def __init__(
        self,
        pipeline_str: str,
        on_frame: Callable[[np.ndarray], None],
        on_error: Callable[[str], None],
    ):
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        Gst.init(None)

        self._Gst = Gst
        self._on_frame = on_frame
        self._on_error = on_error

        self._pipeline = Gst.parse_launch(pipeline_str)
        self._sink = self._pipeline.get_by_name("appsink")
        self._sink.set_property("emit-signals", False)
        self._sink.set_property("max-buffers", 1)
        self._sink.set_property("drop", True)

        ret = self._pipeline.set_state(Gst.State.PLAYING)
        self._ok = ret != Gst.StateChangeReturn.FAILURE

        self._running = True
        self._frame_count: int = 0
        self._last_frame_time: float = 0.0
        self._bus_message: str | None = None

        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        self._bus_thread = threading.Thread(target=self._bus_loop, daemon=True)
        self._bus_thread.start()

    def _capture_loop(self):
        while self._running:
            try:
                sample = self._sink.emit("pull-sample")
            except Exception as e:
                self._on_error(f"pull-sample 예외: {e}")
                break
            if sample is None:
                self._on_error("EOS: 스트림 종료")
                break
            frame = _sample_to_bgr(sample)
            if frame is None:
                continue
            self._frame_count += 1
            self._last_frame_time = time.monotonic()
            self._on_frame(frame)

    def _bus_loop(self):
        bus = self._pipeline.get_bus()
        while self._running:
            msg = bus.timed_pop_filtered(
                100 * 1_000_000,
                self._Gst.MessageType.ERROR
                | self._Gst.MessageType.EOS
                | self._Gst.MessageType.WARNING,
            )
            if msg is None:
                continue
            t = msg.type
            if t == self._Gst.MessageType.ERROR:
                err, dbg = msg.parse_error()
                err_msg = f"ERROR: {err.message} | {dbg}"
                self._bus_message = err_msg
                self._running = False
                self._on_error(err_msg)
                break
            elif t == self._Gst.MessageType.EOS:
                self._bus_message = "EOS"
                self._running = False
                self._on_error("EOS: 파이프라인 종료")
                break
            elif t == self._Gst.MessageType.WARNING:
                err, _ = msg.parse_warning()
                self._bus_message = f"WARNING: {err.message}"

    def is_opened(self) -> bool:
        return self._ok and self._thread.is_alive()

    def status(self) -> dict:
        return {
            "thread_alive": self._thread.is_alive(),
            "frame_count": self._frame_count,
            "last_frame_age": (
                time.monotonic() - self._last_frame_time
                if self._last_frame_time else None
            ),
            "bus_message": self._bus_message,
        }

    def release(self):
        self._running = False
        self._pipeline.set_state(self._Gst.State.NULL)
        self._thread.join(timeout=2.0)
        self._bus_thread.join(timeout=1.0)


class _FfmpegCapture:
    """FFmpeg 기반 캡처. 프레임 도착 시 on_frame 콜백 호출."""

    def __init__(
        self,
        cap: cv2.VideoCapture,
        on_frame: Callable[[np.ndarray], None],
        on_error: Callable[[str], None],
    ):
        self._cap = cap
        self._on_frame = on_frame
        self._on_error = on_error
        self._running = True
        self._frame_count: int = 0
        self._last_frame_time: float = 0.0

        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                self._on_error("FFmpeg: 프레임 읽기 실패")
                break
            self._frame_count += 1
            self._last_frame_time = time.monotonic()
            self._on_frame(frame)

    def is_opened(self) -> bool:
        return self._thread.is_alive()

    def status(self) -> dict:
        return {
            "thread_alive": self._thread.is_alive(),
            "frame_count": self._frame_count,
            "last_frame_age": (
                time.monotonic() - self._last_frame_time
                if self._last_frame_time else None
            ),
            "bus_message": None,
        }

    def release(self):
        self._running = False
        self._cap.release()
        self._thread.join(timeout=2.0)


class RtspImageNode(Node):
    def __init__(self):
        super().__init__("rtsp_image_node")

        self.declare_parameter("rtsp_url", "rtsp://localhost:8554/stream")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("reconnect_delay", 2.0)
        self.declare_parameter("frame_width", 0)
        self.declare_parameter("frame_height", 0)
        self.declare_parameter("backend", "ffmpeg")
        self.declare_parameter("codec", "h264")       # "h264" | "h265"
        self.declare_parameter("decoder", "software")
        self.declare_parameter("gst_pipeline", "")
        self.declare_parameter("gst_latency", 200)
        self.declare_parameter("stale_timeout", 5.0)
        self.declare_parameter("status_interval", 10.0)

        self.rtsp_url = self.get_parameter("rtsp_url").get_parameter_value().string_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.reconnect_delay = self.get_parameter("reconnect_delay").get_parameter_value().double_value
        self.frame_width = self.get_parameter("frame_width").get_parameter_value().integer_value
        self.frame_height = self.get_parameter("frame_height").get_parameter_value().integer_value
        self.backend = self.get_parameter("backend").get_parameter_value().string_value
        self.codec = self.get_parameter("codec").get_parameter_value().string_value
        self.decoder = self.get_parameter("decoder").get_parameter_value().string_value
        self.gst_pipeline = self.get_parameter("gst_pipeline").get_parameter_value().string_value
        self.gst_latency = self.get_parameter("gst_latency").get_parameter_value().integer_value
        self.stale_timeout = self.get_parameter("stale_timeout").get_parameter_value().double_value
        self.status_interval = self.get_parameter("status_interval").get_parameter_value().double_value

        if self.backend not in ("ffmpeg", "gstreamer"):
            self.get_logger().warn(f"알 수 없는 backend '{self.backend}'. 'ffmpeg'로 대체합니다.")
            self.backend = "ffmpeg"
        if self.codec not in _CODECS:
            self.get_logger().warn(f"알 수 없는 codec '{self.codec}'. 'h264'로 대체합니다.")
            self.codec = "h264"
        if self.decoder not in _DECODERS:
            self.get_logger().warn(f"알 수 없는 decoder '{self.decoder}'. 'software'로 대체합니다.")
            self.decoder = "software"

        self.bridge = CvBridge()
        self._cap = None
        self._reconnect_timer = None
        self._reconnect_pending = False  # 중복 재연결 방지

        # 통계
        self._publish_count: int = 0
        self._connect_time: float = 0.0
        self._last_publish_time: float = 0.0

        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        self.create_timer(self.status_interval, self._status_callback)
        self._connect()

    # ------------------------------------------------------------------
    # 연결 / 재연결
    # ------------------------------------------------------------------

    def _build_gst_pipeline_str(self) -> str:
        if self.gst_pipeline:
            return self.gst_pipeline
        return _GST_PIPELINES[(self.decoder, self.codec)].format(
            url=self.rtsp_url, latency=self.gst_latency
        )

    def _connect(self):
        self._publish_count = 0
        self._last_publish_time = 0.0
        self._connect_time = time.monotonic()
        self._reconnect_pending = False
        if self.backend == "gstreamer":
            self._connect_gst()
        else:
            self._connect_ffmpeg()

    def _connect_ffmpeg(self):
        self.get_logger().info(f"FFmpeg 백엔드로 연결: {self.rtsp_url}")
        cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            self.get_logger().error(
                f"FFmpeg 연결 실패. {self.reconnect_delay:.1f}초 후 재시도합니다."
            )
            self._schedule_reconnect()
            return
        if self.frame_width > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        if self.frame_height > 0:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._cap = _FfmpegCapture(cap, self._on_frame, self._on_capture_error)
        self.get_logger().info(
            f"FFmpeg 연결 성공. 해상도={w}x{h}, topic={self.image_topic}"
        )

    def _connect_gst(self):
        pipeline_str = self._build_gst_pipeline_str()
        self.get_logger().info(f"GStreamer 파이프라인:\n  {pipeline_str}")
        try:
            cap = _GstCapture(pipeline_str, self._on_frame, self._on_capture_error)
        except Exception as e:
            self.get_logger().error(f"GStreamer 파이프라인 생성 실패: {e}")
            self._schedule_reconnect()
            return
        if not cap.is_opened():
            self.get_logger().error(
                f"GStreamer 연결 실패 (decoder={self.decoder}). "
                f"{self.reconnect_delay:.1f}초 후 재시도합니다."
            )
            cap.release()
            self._schedule_reconnect()
            return
        self._cap = cap
        self.get_logger().info(
            f"GStreamer 연결 성공. codec={self.codec}, decoder={self.decoder}, "
            f"topic={self.image_topic}, latency={self.gst_latency}ms"
        )

    def _schedule_reconnect(self):
        if self._reconnect_pending:
            return
        self._reconnect_pending = True
        if self._reconnect_timer is not None:
            self._reconnect_timer.cancel()
        self._reconnect_timer = self.create_timer(self.reconnect_delay, self._reconnect_callback)

    def _reconnect_callback(self):
        self._reconnect_timer.cancel()
        self._reconnect_timer = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._connect()

    # ------------------------------------------------------------------
    # 프레임 콜백 (캡처 스레드에서 호출)
    # ------------------------------------------------------------------

    def _on_frame(self, frame: np.ndarray):
        """캡처 스레드에서 프레임 도착 시 즉시 호출."""
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            self.publisher.publish(msg)
            self._publish_count += 1
            self._last_publish_time = time.monotonic()
        except Exception as e:
            self.get_logger().error(f"이미지 퍼블리시 실패: {e}")

    def _on_capture_error(self, reason: str):
        """캡처 스레드에서 에러 발생 시 호출. ROS2 타이머로 재연결 예약."""
        self.get_logger().warn(f"캡처 에러: {reason}. 재연결을 예약합니다.")
        self._schedule_reconnect()

    # ------------------------------------------------------------------
    # 상태 로그 및 stale 감지
    # ------------------------------------------------------------------

    def _status_callback(self):
        elapsed = time.monotonic() - self._connect_time
        if self._last_publish_time > 0:
            last_age = time.monotonic() - self._last_publish_time
            fps = self._publish_count / elapsed if elapsed > 0 else 0.0
            self.get_logger().info(
                f"[상태] backend={self.backend}  published={self._publish_count}프레임  "
                f"실제fps={fps:.1f}  마지막프레임={last_age:.1f}s 전"
            )
        else:
            self.get_logger().warn(
                f"[상태] backend={self.backend}  연결 후 {elapsed:.1f}s 경과  아직 프레임 없음"
            )

        if self._cap is not None:
            st = self._cap.status()
            msg_parts = [
                f"[캡처] thread={'alive' if st['thread_alive'] else 'DEAD'}",
                f"frames={st['frame_count']}",
                f"last_frame_age={st['last_frame_age']:.1f}s"
                if st["last_frame_age"] is not None else "last_frame_age=N/A",
            ]
            if st.get("bus_message"):
                msg_parts.append(f"bus={st['bus_message']}")
            self.get_logger().info("  ".join(msg_parts))

            # stale 감지: 마지막 프레임 이후 stale_timeout 초 초과
            last_age = st["last_frame_age"]
            no_frame_since = (
                last_age if last_age is not None
                else time.monotonic() - self._connect_time
            )
            if no_frame_since > self.stale_timeout:
                self.get_logger().warn(
                    f"프레임 없음 {no_frame_since:.1f}s (기준={self.stale_timeout:.0f}s). 재연결합니다."
                )
                self._schedule_reconnect()

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RtspImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
