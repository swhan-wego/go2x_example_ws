"""
Keyboard Handler Node for ROS2

This node listens to keyboard inputs and publishes the key events to ROS2 topics.
It supports loading key mappings and cooldowns from a YAML configuration file.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time
from pynput import keyboard
import yaml

class KeyboardHandler(Node):
    def __init__(self):
        super().__init__('keyboard_listener')
        
        self.declare_parameter('keymap', './key_config.yaml')
        
        self.pressed_publisher = self.create_publisher(Int32, 'keyboard_pressed', 10)
        self.released_publisher = self.create_publisher(Int32, 'keyboard_released', 10)
        
        self.get_logger().info('Keyboard Listener Node has been started.')
        
        # key_map[키] = (콜백함수, 쿨타임초)
        self.key_map = {}
        
        # last_executed[키] = 마지막 실행 타임스탬프
        self.last_executed = {}
        self.listener = None

    def load_key_map_from_yaml(self, yaml_file_path):
        try:
            with open(yaml_file_path, 'r') as f:
                key_map_config = yaml.safe_load(f)
                if 'key_map' in key_map_config and isinstance(key_map_config['key_map'], list):
                    for item in key_map_config['key_map']:
                        if isinstance(item, dict) and 'key' in item and 'cooldown' in item:
                            key_identifier = str(item['key']).lower()
                            cooldown = item['cooldown']
                            self.add_action(key_identifier, cooldown)
                        else:
                            self.get_logger().warn(f"Invalid item in key_map list: {item}")
                    self.get_logger().info(f"[*] Keymap loaded from {yaml_file_path}")
                else:
                    self.get_logger().warn(f"Keymap section not found or invalid in {yaml_file_path}")
        except FileNotFoundError:
            self.get_logger().error(f"Error: YAML file not found at {yaml_file_path}")
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file {yaml_file_path}: {e}")

    def add_action(self, key_identifier, cooldown=1.0):
        """
        key_identifier: 등록할 키 (예: 'a', 'up', 'space')
        cooldown: 재입력 제한 시간 (초 단위, 기본 1초)
        """
        self.key_map[key_identifier] = cooldown
        self.last_executed[key_identifier] = 0  # 초기값은 0으로 설정
        self.get_logger().info(f"[*] 키 '{key_identifier}' 등록 (쿨타임: {cooldown}초)")

    def _get_key_info(self, key):
        """키 객체에서 식별자(문자열)와 정수 값을 추출"""
        try:
            if hasattr(key, 'char') and key.char is not None:
                return key.char.lower(), ord(key.char)
            else:
                # 특수키 (Key.up, Key.space 등)
                name = key.name.lower()
                # 가능한 경우에는 표준 ASCII 값을 사용하도록 매핑
                special_ascii = {
                    'space': 32,
                    'enter': 13,
                    'tab': 9,
                    'esc': 27,
                    'backspace': 8,
                }
                if name in special_ascii:
                    return name, special_ascii[name]

                # 그 외 특수키는 0-255 범위의 해시로 반환하여
                # 수신측에서 chr()로 복원 가능하도록 함 (충돌 가능성은 낮음)
                return name, hash(name) % 256
        except Exception:
            return None, None

    def _on_press(self, key):
        if key == keyboard.Key.esc:
            self.get_logger().info("ESC pressed, stopping listener.")
            return False

        key_name, key_val = self._get_key_info(key)
        if key_name in self.key_map:
            cooldown = self.key_map[key_name]
            current_time = time.time()
            
            if current_time - self.last_executed[key_name] >= cooldown:
                msg = Int32()
                msg.data = key_val
                self.pressed_publisher.publish(msg)
                self.get_logger().info(f'Pressed: {key_val} (key: {key_name})')
                self.last_executed[key_name] = current_time
            else:
                remaining = cooldown - (current_time - self.last_executed[key_name])
                self.get_logger().info(f"[!] '{key_name}' 키 쿨타임 중... ({remaining:.2f}초 남음)")

    def _on_release(self, key):
        key_name, key_val = self._get_key_info(key)
        if key_name in self.key_map:
            msg = Int32()
            msg.data = key_val
            self.released_publisher.publish(msg)
            self.get_logger().info(f'Released: {key_val} (key: {key_name})')

    def start(self):
        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self.listener.start()
        self.get_logger().info("[!] 스마트 리스너가 시작되었습니다.")

def main(args=None):
    rclpy.init(args=args)

    keyboard_handler_node = KeyboardHandler()

    # Load keymap from YAML file
    yaml_file = keyboard_handler_node.get_parameter('keymap').get_parameter_value().string_value
    keyboard_handler_node.load_key_map_from_yaml(yaml_file)

    keyboard_handler_node.start()

    rclpy.spin(keyboard_handler_node)

    keyboard_handler_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

