"""
finger_utils.py — 손가락 상태 판단 및 제스처 분류 유틸리티 (6.3절)

MediaPipe Hand Landmark 인덱스를 사용하여 손가락 펴짐/접힘 상태를 판단하고,
6가지 손 제스처를 분류합니다.
"""

from collections import deque

# ── MediaPipe Hand Landmark 인덱스 상수 ──────────────────────────────────────
WRIST      = 0
THUMB_CMC  = 1;  THUMB_MCP  = 2;  THUMB_IP   = 3;  THUMB_TIP  = 4
INDEX_MCP  = 5;  INDEX_PIP  = 6;  INDEX_DIP  = 7;  INDEX_TIP  = 8
MIDDLE_MCP = 9;  MIDDLE_PIP = 10; MIDDLE_DIP = 11; MIDDLE_TIP = 12
RING_MCP   = 13; RING_PIP   = 14; RING_DIP   = 15; RING_TIP   = 16
PINKY_MCP  = 17; PINKY_PIP  = 18; PINKY_DIP  = 19; PINKY_TIP  = 20


def get_finger_states(lm) -> dict:
    """
    각 손가락의 펴짐(True) / 접힘(False) 상태를 반환.

    판단 기준:
    - 검지~소지: TIP.y < PIP.y 이면 펴진 상태 (y축 아래가 양수)
    - 엄지: TIP.x > IP.x 이면 펴진 상태 (오른손 기준)

    Args:
        lm: hand_landmarks.landmark 리스트 (21개 요소)

    Returns:
        dict: {"thumb", "index", "middle", "ring", "pinky"} 각 True/False
    """
    index  = lm[INDEX_TIP].y  < lm[INDEX_PIP].y
    middle = lm[MIDDLE_TIP].y < lm[MIDDLE_PIP].y
    ring   = lm[RING_TIP].y   < lm[RING_PIP].y
    pinky  = lm[PINKY_TIP].y  < lm[PINKY_PIP].y
    thumb  = lm[THUMB_TIP].x  > lm[THUMB_IP].x  # 오른손 기준

    return {
        "thumb":  thumb,
        "index":  index,
        "middle": middle,
        "ring":   ring,
        "pinky":  pinky,
    }


def classify_gesture(lm, handedness: str = "Right") -> str:
    """
    손가락 상태로 제스처 문자열을 반환.

    제스처 매핑:
    - ✊ 주먹 (모든 손가락 접힘)     → "stop"
    - ☝️ 검지만 펴기                 → "forward"
    - ✌️ V사인 (검지+중지)           → "backward"
    - 👈 엄지 왼쪽 방향              → "turn_left"
    - 👉 엄지 오른쪽 방향            → "turn_right"
    - 🤚 다섯 손가락 모두 펴기       → "emergency_stop"

    Args:
        lm:          hand_landmarks.landmark 리스트
        handedness:  "Right" 또는 "Left"

    Returns:
        str: "stop" | "forward" | "backward" | "turn_left" | "turn_right"
             | "emergency_stop" | "unknown"
    """
    fs = get_finger_states(lm)

    # 손 너비 기준 (검지 MCP ~ 소지 MCP 거리)
    hand_width = abs(lm[INDEX_MCP].x - lm[PINKY_MCP].x) + 1e-6

    # 엄지가 옆으로 충분히 뻗어 있는지 (손 너비의 40% 이상 이격)
    thumb_offset = lm[THUMB_TIP].x - lm[INDEX_MCP].x
    THUMB_THRESH = 0.4 * hand_width
    if handedness == "Left":
        thumb_left  = thumb_offset >  THUMB_THRESH
        thumb_right = thumb_offset < -THUMB_THRESH
    else:  # Right
        thumb_left  = thumb_offset < -THUMB_THRESH
        thumb_right = thumb_offset >  THUMB_THRESH

    four_fingers = [fs["index"], fs["middle"], fs["ring"], fs["pinky"]]

    # 모두 펴짐 → 긴급 정지 (가장 먼저 검사)
    if all(four_fingers) and fs["thumb"]:
        return "emergency_stop"

    # 모두 접힘 → 정지 / 방향 전환
    if not any(four_fingers):
        if thumb_left:
            return "turn_left"
        if thumb_right:
            return "turn_right"
        return "stop"  # 주먹 (엄지 안 뻗음)

    # 검지만 펴짐 → 전진
    if fs["index"] and not fs["middle"] and not fs["ring"] and not fs["pinky"]:
        return "forward"

    # 검지 + 중지만 펴짐 → 후진
    if fs["index"] and fs["middle"] and not fs["ring"] and not fs["pinky"]:
        return "backward"

    # 약지 + 소지만 펴짐 → 앉기 (sit)
    if not fs["index"] and not fs["middle"] and fs["ring"] and fs["pinky"]:
        return "sit"

    # 소지만 펴짐 → 일어서기 (stand_up)
    if not fs["index"] and not fs["middle"] and not fs["ring"] and fs["pinky"]:
        return "stand_up"

    # 중지 + 약지 + 소지만 펴짐 (검지 없음) → 엎드리기 (lie_down)
    if not fs["index"] and fs["middle"] and fs["ring"] and fs["pinky"]:
        return "lie_down"

    return "unknown"


class GestureSmoothing:
    """
    최근 N 프레임의 다수결(majority vote)로 제스처를 안정화.

    단일 프레임 오인식을 방지하기 위해 슬라이딩 윈도우 내에서
    가장 많이 등장한 제스처를 확정 제스처로 반환합니다.

    Args:
        window: 슬라이딩 윈도우 크기 (기본 10 프레임)
    """

    def __init__(self, window: int = 10):
        self._buf = deque(maxlen=window)

    def update(self, gesture: str) -> str:
        """
        새 제스처를 버퍼에 추가하고 다수결 결과를 반환.

        Args:
            gesture: 현재 프레임 제스처 문자열

        Returns:
            str: 버퍼 내 최다 등장 제스처
        """
        self._buf.append(gesture)
        return max(set(self._buf), key=self._buf.count)

    def reset(self):
        """버퍼 초기화 (모드 전환 시 사용)"""
        self._buf.clear()
