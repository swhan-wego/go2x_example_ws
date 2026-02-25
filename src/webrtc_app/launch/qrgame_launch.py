"""Unitree Go2 QR 게임 런치 파일.

실행되는 노드:
    1. UnitreeRobotNode    - WebRTC ↔ ROS2 브릿지 (webrtc_unitree_ros.py)
    2. KeyboardHandlerNode - 키보드 입력 처리 (webrtc_keyboard_handler)
    3. QRCodeNode          - QR 코드 인식 (webrtc_qrcode)
    4. QRGameNode          - QR 게임 로직 (webrtc_qrgame)
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Find installed webrtc_common package share directory
    common_share = get_package_share_directory("webrtc_common")

    params = os.path.join(common_share, "config", "unitree_params.yaml")

    keymap_candidate = os.path.join(common_share, "config", "key_config.yaml")
    if not os.path.exists(keymap_candidate):
        # fallback to package internal path
        keymap_candidate = os.path.join(common_share, "webrtc_common", "key_config.yaml")

    node_unitree = Node(
        package="webrtc_common",
        executable="webrtc_unitree_ros",
        name="unitree_robot_node",
        output="screen",
        arguments=["--ros-args", "--params-file", params],
    )

    node_keyboard = Node(
        package="webrtc_common",
        executable="webrtc_keyboard_handler",
        name="keyboard_handler_node",
        output="screen",
        arguments=["--ros-args", "-p", f"keymap:={keymap_candidate}"],
    )

    node_qrcode = Node(
        package="webrtc_common",
        executable="webrtc_qrcode",
        name="qrcode_node",
        output="screen",
        arguments=[
            "--ros-args",
            "-p",
            "repeat_same:=true",
            "-p",
            "image_topic:=/front_camera",
        ],
    )

    node_qrgame = Node(
        package="webrtc_app",
        executable="webrtc_qrgame",
        name="qrgame_node",
        output="screen",
    )

    return LaunchDescription(
        [
            LogInfo(msg="Starting unitree + keyboard + qrcode + qrgame nodes"),
            node_unitree,
            node_keyboard,
            node_qrcode,
            node_qrgame,
        ]
    )


if __name__ == "__main__":
    # Allow running the launch file directly with python3
    from launch import LaunchService

    ls = LaunchService(argv=sys.argv[1:])
    ld = generate_launch_description()
    ls.include_launch_description(ld)
    sys.exit(ls.run())
