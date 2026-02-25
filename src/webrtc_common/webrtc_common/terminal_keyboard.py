import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time
import sys
import os
import signal
import threading
import yaml
import readchar

# ============================================================================
# Key Code Mapping Tables
# ============================================================================

# INT32 Code Ranges:
#   0-127      : Standard ASCII (printable chars and common control keys)
#   128-199    : Reserved for future use
#   200-211    : Function keys F1-F12
#   212-219    : Arrow keys and navigation
#   220+       : Other special keys

KEY_CODE_MAP = {
    # Alphabetic and numeric characters (standard ASCII)
    'a': 97, 'b': 98, 'c': 99, 'd': 100, 'e': 101, 'f': 102, 'g': 103, 'h': 104,
    'i': 105, 'j': 106, 'k': 107, 'l': 108, 'm': 109, 'n': 110, 'o': 111, 'p': 112,
    'q': 113, 'r': 114, 's': 115, 't': 116, 'u': 117, 'v': 118, 'w': 119, 'x': 120,
    'y': 121, 'z': 122,
    '0': 48, '1': 49, '2': 50, '3': 51, '4': 52, '5': 53, '6': 54, '7': 55, '8': 56, '9': 57,
    
    # Common control and special keys (standard ASCII)
    'space': 32, 'enter': 13, 'return': 13, 'tab': 9, 'backspace': 8, 'esc': 27,
    'escape': 27, 'delete': 127,
    
    # Symbols
    '!': 33, '@': 64, '#': 35, '$': 36, '%': 37, '^': 94, '&': 38, '*': 42,
    '(': 40, ')': 41, '-': 45, '_': 95, '=': 61, '+': 43, '[': 91, ']': 93,
    '{': 123, '}': 125, '\\': 92, '|': 124, ';': 59, ':': 58, '\'': 39, '"': 34,
    ',': 44, '<': 60, '.': 46, '>': 62, '/': 47, '?': 63, '`': 96, '~': 126,
    
    # Function keys F1-F12 (200-211)
    'f1': 200, 'f2': 201, 'f3': 202, 'f4': 203, 'f5': 204, 'f6': 205,
    'f7': 206, 'f8': 207, 'f9': 208, 'f10': 209, 'f11': 210, 'f12': 211,
    
    # Arrow keys and navigation (212+)
    'up': 212, 'down': 213, 'left': 214, 'right': 215,
    'home': 216, 'end': 217, 'pageup': 218, 'pagedown': 219,
}

# Reverse mapping: INT32 code -> key name
INVERSE_KEY_MAP = {v: k for k, v in KEY_CODE_MAP.items()}

# Special readchar sequences: raw_key_sequence -> (key_name, int_code)
READCHAR_SPECIAL_MAP = {
    # Function keys (various terminal emulator formats)
    '\x1b[11~': ('f1', 200), '\x1b[12~': ('f2', 201), '\x1b[13~': ('f3', 202), '\x1b[14~': ('f4', 203),
    '\x1b[15~': ('f5', 204), '\x1b[17~': ('f6', 205), '\x1b[18~': ('f7', 206), '\x1b[19~': ('f8', 207),
    '\x1b[20~': ('f9', 208), '\x1b[21~': ('f10', 209), '\x1b[23~': ('f11', 210), '\x1b[24~': ('f12', 211),
    
    # SS3 format function keys (some terminals)
    '\x1bOP': ('f1', 200), '\x1bOQ': ('f2', 201), '\x1bOR': ('f3', 202), '\x1bOS': ('f4', 203),
    
    # Arrow keys
    '\x1b[A': ('up', 212), '\x1b[B': ('down', 213), '\x1b[C': ('right', 215), '\x1b[D': ('left', 214),
    
    # Enter/Return variations
    '\r': ('enter', 13), '\n': ('enter', 13),
}

# ============================================================================
# Key Conversion Functions
# ============================================================================

def get_key_code(key_name):
    """
    Convert key name (string) to INT32 code.
    
    Args:
        key_name (str): Key name (e.g., 'a', 'enter', 'f1', 'up')
    
    Returns:
        int: INT32 code, or None if key not found
    """
    return KEY_CODE_MAP.get(key_name.lower())

def get_key_name(key_code):
    """
    Convert INT32 code back to key name.
    
    Args:
        key_code (int): INT32 code
    
    Returns:
        str: Key name, or None if code not found
    """
    return INVERSE_KEY_MAP.get(key_code)

def readchar_to_key(raw_key):
    """
    Convert raw readchar.readkey() output to (key_name, int_code) tuple.
    Handles standard characters, special sequences, and escape codes.
    
    Args:
        raw_key (str): Raw input from readchar.readkey()
    
    Returns:
        tuple: (key_name: str, key_code: int) or (None, None) if unrecognized
    """
    # Check special sequences first (escape codes, multi-char sequences)
    if raw_key in READCHAR_SPECIAL_MAP:
        return READCHAR_SPECIAL_MAP[raw_key]
    
    # Check readchar constants
    if raw_key == readchar.key.ESC or raw_key == '\x1b':
        return ('esc', 27)
    elif raw_key == readchar.key.UP:
        return ('up', 212)
    elif raw_key == readchar.key.DOWN:
        return ('down', 213)
    elif raw_key == readchar.key.RIGHT:
        return ('right', 215)
    elif raw_key == readchar.key.LEFT:
        return ('left', 214)
    
    # Single printable character
    if len(raw_key) == 1:
        key_char = raw_key.lower()
        key_code = get_key_code(key_char)
        if key_code is not None:
            return (key_char, key_code)
        else:
            # Unknown single character - use ASCII code
            return (key_char, ord(raw_key))
    
    # Unknown multi-character sequence
    return (None, None)

# ============================================================================

class KeyboardHandler(Node):
    def __init__(self):
        super().__init__('terminal_keyboard')
        
        self.declare_parameter('default_cooldown', 0.5)
        self.default_cooldown = float(self.get_parameter('default_cooldown').get_parameter_value().double_value)
        
        self.pressed_publisher = self.create_publisher(Int32, 'keyboard_pressed', 10)
        
        self.get_logger().info('Keyboard Listener Node has been started.')
        
        # last_executed[키] = 마지막 실행 타임스탠프
        self.last_executed = {}



    def start(self):
        # Start a background thread to read stdin for key presses (works over SSH)
        self._stop_event = threading.Event()

        def stdin_thread():
            self.get_logger().info("[!] Terminal input listener started (SSH-friendly, using readchar)")

            while not self._stop_event.is_set():
                try:
                    k = readchar.readkey()
                except Exception as e:
                    self.get_logger().error(f"readkey error: {e}")
                    break

                # Convert raw readchar input to key name and INT32 code
                name, val = readchar_to_key(k)

                if name is not None:
                    # handle ESC -> shutdown
                    if name == 'esc':
                        self.get_logger().info("ESC pressed, stopping listener and sending SIGINT.")
                        self._stop_event.set()
                        try:
                            os.kill(os.getpid(), signal.SIGINT)
                        except Exception:
                            pass
                        return

                    # emulate press
                    try:
                        current_time = time.time()
                        last_time = self.last_executed.get(name, 0)
                        if current_time - last_time >= self.default_cooldown:
                            msg = Int32()
                            msg.data = val
                            self.pressed_publisher.publish(msg)
                            self.get_logger().info(f'Pressed: {val} (key: {name})')
                            self.last_executed[name] = current_time
                        else:
                            remaining = self.default_cooldown - (current_time - last_time)
                            self.get_logger().info(f"[!] '{name}' 키 쿨타임 중... ({remaining:.2f}초 남음)")
                    except Exception as e:
                        self.get_logger().error(f"Error handling key input: {e}")

        self._stdin_thread = threading.Thread(target=stdin_thread, daemon=True)
        self._stdin_thread.start()

def main(args=None):
    rclpy.init(args=args)

    keyboard_handler_node = KeyboardHandler()
    keyboard_handler_node.start()

    try:
        rclpy.spin(keyboard_handler_node)
    except KeyboardInterrupt:
        # Interrupted by SIGINT (e.g., ESC -> SIGINT). We'll clean up below.
        pass
    finally:
        # signal stdin thread to stop and join
        try:
            if hasattr(keyboard_handler_node, '_stop_event'):
                keyboard_handler_node._stop_event.set()
            if hasattr(keyboard_handler_node, '_stdin_thread') and keyboard_handler_node._stdin_thread.is_alive():
                keyboard_handler_node._stdin_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            keyboard_handler_node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

