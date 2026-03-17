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
        executable="object_tracking_node",
        name="object_tracking_node",
        output="screen",
        additional_env={"PYTHONPATH": _pythonpath},
        parameters=[
            # 입력 위치 토픽 (object_yolo_node 출력과 일치)
            {"object_pos_topic": "/object_position"},
            # 로봇 이동 명령 토픽
            {"cmd_vel_topic":    "/cmd_vel"},
            # X축 정렬 허용 오차 (0.01~0.5)
            {"x_tolerance":      0.1},
            # 전진 정지 Y 임계값 — y > (1 - y_stop_range) 이면 도달 (0.01~0.5)
            {"y_stop_range":     0.25},
            # 제어 루프 주기 초 (0.01~1.0)
            {"timer_period":     0.1},
            # 최대 이동/회전 속도 (0.01~1.0)
            {"max_speed":        0.3},
            # 위치 수신 타임아웃 초
            {"pos_timeout":      1.0},
        ],
    )

    return LaunchDescription([node])


if __name__ == "__main__":
    from launch import LaunchService

    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
