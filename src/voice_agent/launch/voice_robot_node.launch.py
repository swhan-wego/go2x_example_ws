import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node

# uv venv의 site-packages를 PYTHONPATH에 추가
# AMENT_PREFIX_PATH 첫 항목 → install/<pkg> → install → workspace 루트
_prefix = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)[0]
_workspace = os.path.dirname(os.path.dirname(_prefix)) if _prefix else os.path.expanduser("~")
_VENV_SITE = os.path.join(_workspace, ".venv", "lib", "python3.10", "site-packages")

_pythonpath = _VENV_SITE + os.pathsep + os.environ.get("PYTHONPATH", "")


def generate_launch_description():
    node = Node(
        package="voice_agent",
        executable="voice_robot_node",
        name="voice_robot_node",
        output="screen",
        additional_env={"PYTHONPATH": _pythonpath},
        parameters=[
            {"stt_model": "small"},
            {"record_duration": 3.0},
            {"linear_speed": 0.3},
            {"angular_speed": 0.5},
        ],
    )

    return LaunchDescription([node])


if __name__ == "__main__":
    from launch import LaunchService

    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
