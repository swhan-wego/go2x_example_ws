# Go2 WebRTC 예제 워크스페이스

간단한 설명

- 이 워크스페이스는 Unitree Go2 로봇과 연동하는 WebRTC ↔ ROS2 브릿지와 예제 애플리케이션을 포함합니다.
- 주요 구성요소: `webrtc_common` (공유 노드/유틸리티), `webrtc_app` (예제 노드·런치)
- ROS2 Humble 기준

## 주요 경로

- `src/webrtc_common/` - 공통 노드 및 설정 (`config/`에 `unitree_params.yaml`, `key_config.yaml` 등)
- `src/webrtc_app/` - 예제 애플리케이션 노드 및 `launch/` (qrgame_launch.py, color_track_basic.py 등)

## 빠른 시작 (개발자용)

1. 개발 도구 `uv` 사용 (권장)

```bash
# 가상환경 생성 및 활성화 (uv 사용, 기본)
uv .venv
source .venv/bin/activate
uv pip install setuptools==58.2.0 wheel colcon-common-extensions
```

2. 의존성/패키지 설치 (편집 모드)

```bash
# 워크스페이스 루트에서
uv pip install -e src/webrtc_common
uv pip install -e src/webrtc_app
```

3. (선택) ROS2 ament/colcon 방식으로 빌드

```bash
# ROS2 환경을 먼저 source 한 뒤
# source /opt/ros/<distro>/setup.bash
colcon build
source install/setup.bash
```

- 설명: `--symlink-install`은 빌드 결과 대신 패키지 소스 디렉토리를 설치 공간에 심링크로 연결합니다. Python 패키지의 경우 코드 수정 후 재설치 없이 즉시 반영되어 반복 개발 속도가 빨라집니다.
- 주의: C 확장 모듈이 있는 패키지나 배포/프로덕션 설치에는 권장되지 않습니다. 문제 발생 시 일반 빌드(`colcon build`)로 돌아가세요.

## 런치 및 실행 예

- Go2 기본 런치:

```bash
cd go2x_example_ws/
source .venv/bin/activate
source install/setup.bash
source ./setup_netif.sh 
ros2 launch webrtc_common unitree_launch.py
```

- 패키지 런치 (권장):

```bash
ros2 launch webrtc_common unitree_launch.py
ros2 launch webrtc_app qrgame_launch.py
ros2 launch webrtc_app color_track_basic.py
```

- 개별 노드(콘솔 스크립트) 실행 예:

```bash
# webrtc_common에 등록된 콘솔 스크립트 예
webrtc_unitree_ros --ros-args --params-file src/webrtc_common/config/unitree_params.yaml
webrtc_keyboard_handler --ros-args -p keymap:=src/webrtc_common/config/key_config.yaml

# webrtc_app에 등록된 콘솔 스크립트 예
color_block_tracker --ros-args -p image_topic:=/front_camera
```

## 테스트 방법

### ROS 토픽 발행
전진/후진
```
# 전진
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 후진
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

좌/우 이동
```
# 좌로 이동
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 우로 이동
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: -0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

회전
```
# 시계 방향 회전
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}'

# 반시계 방향 회전
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
```

정지
```
# 정지
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```
연속 발행 (지속 제어가 필요한 경우)
```
# --once 대신 -r 10 옵션으로 10Hz 지속 발행
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

> --once: 한 번만 발행 후 종료\
> -r 10: 10Hz로 지속 발행 (Ctrl+C로 종료)


## 구성 및 리소스

- QR 모델 파일: 워크스페이스 루트의 `models/` 폴더에 위치해야 합니다 (참조: `src/webrtc_common/webrtc_common/qrcode.py`).
- 키맵 파일: `src/webrtc_common/config/key_config.yaml` (패키지 내부에 fallback copy가 존재함)

## 문제 해결 팁

- `ros2 launch`에서 노드(또는 패키지)를 찾지 못하면, 패키지가 설치되었는지(`uv pip install -e`) 또는 `colcon build` 후 `source install/setup.bash`를 했는지 확인하세요.
- OpenCV GUI 창을 사용하려면 X 서버가 필요합니다 (원격 환경인 경우 `XVFB` 같은 가상 프레임버퍼 사용).
- 빌드 문제(예: setuptools 관련)는 가상환경에서 `uv pip install -U setuptools packaging wheel` 로 해결할 수 있습니다.

### CycloneDDS 네트워크 인터페이스 설정 확인

```sh
(go2x_example_ws) wego@wego-Thin-15-B13VE:~/go2x_example_ws$ echo "$CYCLONEDDS_URI"
<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="wlx705dccf78722" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>
```

### CycloneDDS 네트워크 인터페이스 설정 스크립트 실행

```ini
(go2x_example_ws) wego@wego-Thin-15-B13VE:~/go2x_example_ws$ source ./setup_netif.sh 

========================================
🔧 CycloneDDS 네트워크 인터페이스 설정
========================================

📍 사용 가능한 네트워크 인터페이스:
  [1] enp4s0 (IP: )
  [2] wlo1 (IP: 192.168.0.120)
  [3] wlx705dccf78722 (IP: 192.168.12.102)
  [4] docker0 (IP: 172.17.0.1)
  [5] br-c86d8f9dd659 (IP: 172.18.0.1)
  [6] enx00e04c45e449 (IP: )

👉 사용할 인터페이스 번호를 입력하세요 (10초 타임아웃):

번호 입력: 3

========================================
✅ 설정 완료!
========================================

📡 선택된 인터페이스: wlx705dccf78722
🔗 CYCLONEDDS_URI 설정됨

현재 환경변수:
  CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="wlx705dccf78722" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'

💡 이 설정은 현재 쉘 세션에만 적용됩니다.
   영구적으로 적용하려면 ~/.bashrc 또는 ~/.zshrc에 추가하세요.
```

### 시스템 site-packages 사용

```bash
# 또는 시스템 site-packages를 포함해서 가상환경 생성
uv venv --system-site-packages .venv
source .venv/bin/activate
uv pip install setuptools==58.2.0 wheel colcon-common-extensions
```

> `--system-site-packages` 옵션은 시스템에 설치된 패키지(예: ROS2 관련 라이브러리)를 가상환경에서 재사용해야 할 때 유용합니다. 보안/격리 요구사항에 따라 사용 여부를 결정하세요.

### 빠른 개발: symlink(심링크) 설치 옵션

```bash
# 개발 중에 소스 변경을 즉시 반영하려면 symlink 설치를 사용하세요.
colcon build --symlink-install
source install/setup.bash
```

## 유지관리자

- wego (sw.han@wego-robotics.com)