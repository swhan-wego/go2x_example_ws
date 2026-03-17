import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    venv_python = os.path.join(os.getcwd(), ".venv", "bin", "python")

    pkg_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

    keymap_candidate = os.path.join(pkg_root, "config", "key_config.yaml")
    if not os.path.exists(keymap_candidate):
        keymap_candidate = os.path.join(pkg_root, "webrtc_common", "key_config.yaml")

    node_keyboard_handler = Node(
        package="webrtc_common",
        executable="webrtc_keyboard_handler",
        name="keyboard_handler_node",
        output="screen",
        arguments=["--ros-args", "-p", f"keymap:={keymap_candidate}"],
        prefix=[venv_python],
    )

    return LaunchDescription([node_keyboard_handler])


if __name__ == "__main__":
    from launch import LaunchService

    ls = LaunchService(argv=sys.argv[1:])
    ld = generate_launch_description()
    ls.include_launch_description(ld)
    sys.exit(ls.run())
