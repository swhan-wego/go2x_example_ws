import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node

# decoder 옵션:
#   "software"     : CPU 디코딩 (avdec_h264) - 항상 동작
#   "vaapi"        : Intel/AMD GPU, GStreamer 1.20+ va 플러그인 (vah264dec) - 현재 시스템 사용 가능
#   "vaapi_legacy" : Intel/AMD GPU, 구버전 vaapi 플러그인 (vaapidecodebin)
#                    설치: sudo apt install gstreamer1.0-vaapi
#   "nvdec"        : NVIDIA GPU (nvh264dec)
#                    설치: sudo apt install gstreamer1.0-plugins-bad

# uv venv의 site-packages를 PYTHONPATH에 추가
_prefix = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)[0]
_workspace = (
    os.path.dirname(os.path.dirname(_prefix)) if _prefix else os.path.expanduser("~")
)
_VENV_SITE = os.path.join(_workspace, ".venv", "lib", "python3.10", "site-packages")

_pythonpath = _VENV_SITE + os.pathsep + os.environ.get("PYTHONPATH", "")


def generate_launch_description():
    node = Node(
        package="image_agent",
        executable="rtsp_image_node",
        name="rtsp_image_node",
        output="screen",
        additional_env={"PYTHONPATH": _pythonpath},
        parameters=[
            {"rtsp_url": "rtsp://wegocam:yougowego@192.168.0.80:554/stream2"},
            {"image_topic": "/image_raw"},
            {"reconnect_delay": 2.0},
            # 백엔드 선택: "ffmpeg" | "gstreamer"
            {"backend": "gstreamer"},
            # FFmpeg 전용 해상도 (0 = 원본)
            {"frame_width": 0},
            {"frame_height": 0},
            # GStreamer 전용: 코덱 선택 "h264" | "h265"
            {"codec": "h264"},
            # GStreamer 전용: 디코더 선택 "software" | "vaapi" | "vaapi_legacy" | "nvdec"
            {"decoder": "software"},
            # GStreamer 전용: 직접 파이프라인 지정 (비어있으면 자동 생성)
            {"gst_pipeline": ""},
            # GStreamer 전용: rtspsrc latency (ms) — 네트워크 지터 버퍼. 0이면 stall 위험
            {"gst_latency": 200},
        ],
    )

    return LaunchDescription([node])


if __name__ == "__main__":
    from launch import LaunchService

    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
