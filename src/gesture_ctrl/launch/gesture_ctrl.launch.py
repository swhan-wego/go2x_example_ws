"""
gesture_ctrl.launch.py — 제스처 컨트롤 통합 런치 파일 (6.5절)

두 ROS2 노드를 동시에 실행합니다:
  1. gesture_node   — 웹캠 제스처 인식 → /gesture_cmd 퍼블리시
  2. go2_controller — /gesture_cmd 수신 → Go2 SDK 호출

실행:
  ros2 launch gesture_ctrl gesture_ctrl.launch.py
  ros2 launch gesture_ctrl gesture_ctrl.launch.py camera_index:=1 smooth_window:=15
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # uv 가상환경 내의 python 경로 설정
    # 보통 프로젝트 루트의 .venv/bin/python (Linux/macOS)
    venv_python = os.path.join(os.getcwd(), ".venv", "bin", "python")

    return LaunchDescription(
        [
            # ── 런치 인수 선언 ─────────────────────────────────────────────────
            DeclareLaunchArgument(
                "camera_index",
                default_value="0",
                description="웹캠 장치 번호 (/dev/video<N>)",
            ),
            DeclareLaunchArgument(
                "smooth_window",
                default_value="10",
                description="제스처 스무딩 윈도우 크기 (프레임 수)",
            ),
            # ── 제스처 인식 노드 ───────────────────────────────────────────────
            Node(
                package="gesture_ctrl",
                executable="gesture_node",
                name="gesture_node",
                output="screen",
                prefix=venv_python,
                parameters=[
                    {
                        "camera_index": LaunchConfiguration("camera_index"),
                        "smooth_window": LaunchConfiguration("smooth_window"),
                        "publish_rate": 10.0,
                    }
                ],
            ),
        ]
    )
