# ROS 2 Python 패키지 새로 추가하기

이 문서는 `/home/wego/go2x_example_ws` 워크스페이스에서 ROS 2 Python 패키지를 추가하는 기본 절차를 설명합니다.

## 1) `src` 디렉터리로 이동

```bash
cd /home/wego/go2x_example_ws/src
```

## 2) 패키지 생성

예시 패키지 이름: `my_py_pkg`

```bash
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy std_msgs
```

생성 후 기본 구조(예시):

```text
my_py_pkg/
  package.xml
  setup.py
  setup.cfg
  resource/my_py_pkg
  my_py_pkg/
    __init__.py
```

## 3) 노드 파일 작성

파일 생성: `my_py_pkg/my_py_pkg/my_node.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Hello from my_node')


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

실행 권한 부여:

```bash
chmod +x my_py_pkg/my_py_pkg/my_node.py
```

## 4) `setup.py`에 실행 엔트리 등록

`entry_points`에 아래를 추가/확인:

```python
entry_points={
    'console_scripts': [
        'my_node = my_py_pkg.my_node:main',
    ],
},
```

## 5) 빌드

워크스페이스 루트에서:

```bash
cd /home/wego/go2x_example_ws
colcon build --packages-select my_py_pkg
```

## 6) 환경 적용 및 실행

```bash
source install/setup.bash
ros2 run my_py_pkg my_node
```

## 자주 놓치는 포인트

- `my_py_pkg/my_py_pkg/__init__.py` 파일이 있어야 Python 패키지 인식이 정상 동작합니다.
- 새 터미널을 열 때마다 `source install/setup.bash`를 다시 실행해야 합니다.
- 의존성 변경 시 `package.xml`과 `setup.py`(또는 `setup.cfg`)를 함께 맞추는 것이 안전합니다.