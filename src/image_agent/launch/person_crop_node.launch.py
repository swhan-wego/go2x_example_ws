import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node

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
        executable="person_crop_node",
        name="person_crop_node",
        output="screen",
        additional_env={"PYTHONPATH": _pythonpath},
        parameters=[
            {"image_topic": "/front_camera"},
            {"person_topic": "/person_image"},
            # HOG 피라미드 스케일 (1.01~1.5, 작을수록 정밀하지만 느림)
            {"scale": 1.05},
            # 슬라이딩 윈도우 스트라이드 px
            {"win_stride": 8},
            # 바운딩 박스 주변 여백 비율 (0.1 = 10%)
            {"padding": 0.1},
            # 최소 검출 면적 (px²) — 너무 작은 오탐 제거
            {"min_area": 3000},
            # NMS overlap 임계값
            {"nms_threshold": 0.65},
        ],
    )

    return LaunchDescription([node])


if __name__ == "__main__":
    from launch import LaunchService

    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
