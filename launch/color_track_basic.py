"""Unitree Go2 전체 스택 런치 파일.

실행되는 노드:
  1. UnitreeRobotNode    - WebRTC ↔ ROS2 브릿지 (webrtc_unitree_ros.py)
  2. KeyboardHandlerNode - 키보드 입력 처리 (keyboard_handler.py)
  3. ColorBlockTrackerNode  - 색깔 블록 검출 (color_block_tracker.py)
  4. ColorBlockVisualizerNode - 검출 결과 시각화 (color_block_visualizer.py)

실행 방법:
  python3 launch/go2_full.launch.py
  또는
  ros2 launch <package> go2_full.launch.py
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo, GroupAction
from launch.substitutions import LocalSubstitution


def generate_launch_description():
    # ── 경로 설정 ────────────────────────────────────────────────────
    repo_root  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    libs_dir   = os.path.join(repo_root, 'go2_libs')
    config_dir = os.path.join(repo_root, 'config')

    # 스크립트 경로
    script_unitree    = os.path.join(libs_dir, 'webrtc_unitree_ros.py')
    script_keyboard   = os.path.join(libs_dir, 'keyboard_handler.py')
    script_tracker    = os.path.join(libs_dir, 'color_block_tracker.py')
    script_visualizer = os.path.join(libs_dir, 'color_block_visualizer.py')

    # 파라미터 파일 경로
    params_unitree  = os.path.join(config_dir, 'unitree_params.yaml')
    params_tracker  = os.path.join(config_dir, 'tracker_params.yaml')
    params_vis      = os.path.join(config_dir, 'visualizer_params.yaml')

    # 키맵 파일: repo_root 우선, 없으면 go2_libs 사용
    keymap = os.path.join(repo_root, 'key_config.yaml')
    if not os.path.exists(keymap):
        keymap = os.path.join(libs_dir, 'key_config.yaml')

    # ── 파일 존재 확인 ───────────────────────────────────────────────
    required_files = {
        "UnitreeRobotNode 스크립트":      script_unitree,
        "KeyboardHandlerNode 스크립트":   script_keyboard,
        "ColorBlockTrackerNode 스크립트": script_tracker,
        "ColorBlockVisualizerNode 스크립트": script_visualizer,
        "Unitree 파라미터 파일":           params_unitree,
        "키맵 파일":                       keymap,
    }
    missing = [f"{name}: {path}" for name, path in required_files.items()
               if not os.path.exists(path)]
    if missing:
        print("[ERROR] 다음 파일이 누락되었습니다:")
        for m in missing:
            print(f"  ✗ {m}")
        sys.exit(1)

    # tracker / visualizer 파라미터 파일은 선택적
    tracker_params_args = (
        ['--params-file', params_tracker]
        if os.path.exists(params_tracker) else []
    )
    vis_params_args = (
        ['--params-file', params_vis]
        if os.path.exists(params_vis) else []
    )

    # ── 프로세스 정의 ────────────────────────────────────────────────

    # 1. UnitreeRobotNode: WebRTC 연결 및 ROS2 토픽 브릿지
    #    가장 먼저 실행되어야 하며, WebRTC 연결 수립 후 카메라 토픽을 발행
    proc_unitree = ExecuteProcess(
        cmd=[sys.executable, script_unitree,
             '--ros-args', '--params-file', params_unitree],
        output='screen',
        name='unitree_robot_node',
    )

    # 2. KeyboardHandlerNode: 키보드 입력을 ROS2 토픽으로 변환
    proc_keyboard = ExecuteProcess(
        cmd=[sys.executable, script_keyboard,
             '--ros-args', '-p', f"keymap:={keymap}"],
        output='screen',
        name='keyboard_handler_node',
    )

    # 3. ColorBlockTrackerNode: 카메라 이미지에서 색깔 블록 검출
    #    UnitreeRobotNode가 카메라 토픽을 발행하기까지 약 3초 대기
    proc_tracker = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="[ColorBlockTracker] 노드 시작..."),
            ExecuteProcess(
                cmd=[sys.executable, script_tracker,
                     '--ros-args',
                     '-p', 'image_topic:=/front_camera',
                     '-p', 'output_topic:=/color_blocks',
                     '-p', 'draw_debug:=false',
                     *tracker_params_args],
                output='screen',
                name='color_block_tracker_node',
            ),
        ]
    )

    # 4. ColorBlockVisualizerNode: 검출 결과를 이미지에 시각화
    #    TrackerNode가 먼저 실행된 후 시작 (추가 1초 대기)
    proc_visualizer = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="[ColorBlockVisualizer] 노드 시작..."),
            ExecuteProcess(
                cmd=[sys.executable, script_visualizer,
                     '--ros-args',
                     '-p', 'image_topic:=/front_camera',
                     '-p', 'blocks_topic:=/color_blocks',
                     '-p', 'output_topic:=/color_blocks/visualized',
                     '-p', 'show_crosshair:=true',
                     *vis_params_args],
                output='screen',
                name='color_block_visualizer_node',
            ),
        ]
    )

    return LaunchDescription([
        LogInfo(msg="=" * 55),
        LogInfo(msg="  Unitree Go2 전체 스택 시작"),
        LogInfo(msg=f"  repo_root : {repo_root}"),
        LogInfo(msg=f"  keymap    : {keymap}"),
        LogInfo(msg="=" * 55),
        proc_unitree,
        proc_keyboard,
        proc_tracker,
        proc_visualizer,
    ])


if __name__ == '__main__':
    from launch import LaunchService

    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())