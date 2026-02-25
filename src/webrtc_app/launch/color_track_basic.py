"""Unitree Go2 색상 추적 런치 파일.

실행되는 노드:
    1. UnitreeRobotNode      - WebRTC ↔ ROS2 브릿지 (webrtc_unitree_ros.py)
    2. KeyboardHandlerNode   - 키보드 입력 처리 (keyboard_handler.py)
    3. ColorBlockTrackerNode - 색깔 블록 검출 (color_block_tracker.py)
    4. ColorBlockVisualizerNode - 검출 결과 시각화 (color_block_visualizer.py)
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LocalSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Find installed package share directories ───────────────────────
    common_share = get_package_share_directory("webrtc_common")
    app_share = get_package_share_directory("webrtc_app")

    # 파라미터 및 스크립트 경로 (installed layout)
    config_dir = os.path.join(common_share, "config")
    app_code_dir = os.path.join(app_share, "webrtc_app")

    params_unitree = os.path.join(config_dir, "unitree_params.yaml")
    params_tracker = os.path.join(config_dir, "tracker_params.yaml")
    params_vis = os.path.join(config_dir, "visualizer_params.yaml")

    # 키맵 파일: webrtc_common/config 우선, 없으면 패키지 내부 사용
    keymap = os.path.join(config_dir, "key_config.yaml")
    if not os.path.exists(keymap):
        keymap = os.path.join(common_share, "webrtc_common", "key_config.yaml")

    # ── 파일 존재 확인 ───────────────────────────────────────────────
    required_files = {
        "Unitree 파라미터 파일": params_unitree,
        "키맵 파일": keymap,
    }
    missing = [
        f"{name}: {path}"
        for name, path in required_files.items()
        if not os.path.exists(path)
    ]
    if missing:
        print("[ERROR] 다음 파일이 누락되었습니다:")
        for m in missing:
            print(f"  ✗ {m}")
        sys.exit(1)

    # tracker / visualizer 파라미터 파일은 선택적
    tracker_params_args = (
        ["--params-file", params_tracker] if os.path.exists(params_tracker) else []
    )
    vis_params_args = (
        ["--params-file", params_vis] if os.path.exists(params_vis) else []
    )

    # ── 프로세스 정의 ────────────────────────────────────────────────

    node_unitree = Node(
        package="webrtc_common",
        executable="webrtc_unitree_ros",
        name="unitree_robot_node",
        output="screen",
        arguments=["--ros-args", "--params-file", params_unitree],
    )

    node_keyboard = Node(
        package="webrtc_common",
        executable="webrtc_keyboard_handler",
        name="keyboard_handler_node",
        output="screen",
        arguments=["--ros-args", "-p", f"keymap:={keymap}"],
    )

    # 3. ColorBlockTrackerNode: 카메라 이미지에서 색깔 블록 검출
    #    UnitreeRobotNode가 카메라 토픽을 발행하기까지 약 3초 대기
    proc_tracker = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="[ColorBlockTracker] 노드 시작..."),
            Node(
                package="webrtc_app",
                executable="color_block_tracker",
                name="color_block_tracker_node",
                output="screen",
                arguments=[
                    "--ros-args",
                    "-p",
                    "image_topic:=/front_camera",
                    "-p",
                    "output_topic:=/color_blocks",
                    "-p",
                    "draw_debug:=false",
                    *tracker_params_args,
                ],
            ),
        ],
    )

    # 4. ColorBlockVisualizerNode: 검출 결과를 이미지에 시각화
    #    TrackerNode가 먼저 실행된 후 시작 (추가 1초 대기)
    proc_visualizer = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="[ColorBlockVisualizer] 노드 시작..."),
            Node(
                package="webrtc_app",
                executable="color_block_visualizer",
                name="color_block_visualizer_node",
                output="screen",
                arguments=[
                    "--ros-args",
                    "-p",
                    "image_topic:=/front_camera",
                    "-p",
                    "blocks_topic:=/color_blocks",
                    "-p",
                    "output_topic:=/color_blocks/visualized",
                    "-p",
                    "show_crosshair:=true",
                    *vis_params_args,
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            LogInfo(msg="=" * 55),
            LogInfo(msg="  Unitree Go2 색상 추적 시작"),
            LogInfo(msg=f"  app_code_dir  : {app_code_dir}"),
            LogInfo(msg=f"  keymap    : {keymap}"),
            LogInfo(msg="=" * 55),
            node_unitree,
            node_keyboard,
            proc_tracker,
            proc_visualizer,
        ]
    )


if __name__ == "__main__":
    from launch import LaunchService

    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
