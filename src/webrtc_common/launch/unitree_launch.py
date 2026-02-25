import os
import sys
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    pkg_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    params = os.path.join(pkg_root, "config", "unitree_params.yaml")

    keymap_candidate = os.path.join(pkg_root, "config", "key_config.yaml")
    if not os.path.exists(keymap_candidate):
        keymap_candidate = os.path.join(pkg_root, "webrtc_common", "key_config.yaml")

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

    return LaunchDescription([LogInfo(msg="Starting unitree + keyboard nodes"), node_unitree, node_keyboard])


if __name__ == "__main__":
    # Allow running the launch file directly with python3
    from launch import LaunchService

    ls = LaunchService(argv=sys.argv[1:])
    ld = generate_launch_description()
    ls.include_launch_description(ld)
    sys.exit(ls.run())
