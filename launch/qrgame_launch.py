import os
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    script = os.path.join(repo_root, 'go2_libs', 'webrtc_unitree_ros.py')
    params = os.path.join(repo_root, 'config', 'unitree_params.yaml')

    keyboard_script = os.path.join(repo_root, 'go2_libs', 'keyboard_handler.py')
    # Prefer root key_config.yaml if present, otherwise use go2_libs/key_config.yaml
    keymap_candidate = os.path.join(repo_root, 'key_config.yaml')
    if not os.path.exists(keymap_candidate):
        keymap_candidate = os.path.join(repo_root, 'go2_libs', 'key_config.yaml')
        
    qrcode_script = os.path.join(repo_root, 'go2_libs', 'qrcode.py')
    
    qrgame_script = os.path.join(repo_root, 'webrtc_ros_qrgame.py')

    cmd_unitree = [sys.executable, script, '--ros-args', '--params-file', params]
    cmd_keyboard = [sys.executable, keyboard_script, '--ros-args', '-p', f"keymap:={keymap_candidate}"]
    cmd_qrcode = [sys.executable, qrcode_script, '--ros-args', '-p', 'repeat_same:=true', '-p', 'image_topic:=/front_camera']
    cmd_qrgame = [sys.executable, qrgame_script]

    # Prepare environment for child processes with RMW_IMPLEMENTATION unset
    env = os.environ.copy()
    if 'RMW_IMPLEMENTATION' in env:
        del env['RMW_IMPLEMENTATION']

    return LaunchDescription([
        ExecuteProcess(cmd=cmd_unitree, output='screen', env=env),
        ExecuteProcess(cmd=cmd_keyboard, output='screen', env=env),
        ExecuteProcess(cmd=cmd_qrcode, output='screen', env=env),
        ExecuteProcess(cmd=cmd_qrgame, output='screen', env=env),
    ])


if __name__ == '__main__':
    # Allow running the launch file directly with python3
    from launch import LaunchService

    ls = LaunchService(argv=sys.argv[1:])
    ld = generate_launch_description()
    ls.include_launch_description(ld)
    sys.exit(ls.run())
