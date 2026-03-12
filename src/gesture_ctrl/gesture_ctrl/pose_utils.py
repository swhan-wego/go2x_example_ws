"""
pose_utils.py — 포즈 제스처 분류 유틸리티 (6.4절)

MediaPipe Pose Landmarks(33개)를 사용하여 신체 자세를 분류하고,
Go2 로봇의 자세 명령(sit, stand_up, lie_down)으로 매핑합니다.
"""

import mediapipe as mp

PoseLM = mp.solutions.pose.PoseLandmark


def classify_pose(lm) -> str:
    """
    Pose Landmarks로 신체 자세 명령을 분류.

    포즈 매핑:
    - 양팔 T자 (어깨 높이 수평)  → "forward"
    - 왼팔만 머리 위로 올리기    → "sit"
    - 오른팔만 머리 위로 올리기  → "stand_up"
    - 양팔 허벅지 아래로 내리기  → "lie_down"
    - 차렷 자세 (기본)           → "idle"

    판단 기준은 어깨-엉덩이 거리(body_h)로 정규화하여
    카메라 거리에 무관하게 동일한 임계값을 적용합니다.

    Args:
        lm: pose_landmarks.landmark 리스트 (33개 요소)

    Returns:
        str: "forward" | "sit" | "stand_up" | "lie_down" | "idle" | "unknown"
    """
    l_shoulder = lm[PoseLM.LEFT_SHOULDER]
    r_shoulder = lm[PoseLM.RIGHT_SHOULDER]
    l_wrist    = lm[PoseLM.LEFT_WRIST]
    r_wrist    = lm[PoseLM.RIGHT_WRIST]
    l_hip      = lm[PoseLM.LEFT_HIP]
    r_hip      = lm[PoseLM.RIGHT_HIP]

    # 어깨 중심 y와 엉덩이 중심 y
    shoulder_y = (l_shoulder.y + r_shoulder.y) / 2
    hip_y      = (l_hip.y + r_hip.y) / 2

    # 신체 높이 (카메라 거리 정규화용) — 0 나눗셈 방지
    body_h = abs(hip_y - shoulder_y) + 1e-6

    # ── 팔 올리기 판단 ──────────────────────────────────────────────────────
    # 손목이 어깨보다 body_h * 0.3 이상 위에 있으면 "올린 상태"
    left_arm_up  = (l_wrist.y < l_shoulder.y - 0.3 * body_h)
    right_arm_up = (r_wrist.y < r_shoulder.y - 0.3 * body_h)

    # ── T자 자세 판단 ────────────────────────────────────────────────────────
    # 조건: 손목 y ≈ 어깨 y (±15% body_h), 손목이 어깨 바깥쪽
    l_arm_horizontal = abs(l_wrist.y - l_shoulder.y) < 0.15 * body_h
    r_arm_horizontal = abs(r_wrist.y - r_shoulder.y) < 0.15 * body_h
    l_wrist_outside  = l_wrist.x < l_shoulder.x  # 왼쪽 바깥 (x 작을수록 왼쪽)
    r_wrist_outside  = r_wrist.x > r_shoulder.x  # 오른쪽 바깥
    t_pose = (l_arm_horizontal and r_arm_horizontal and
              l_wrist_outside and r_wrist_outside)

    # ── 양팔 내리기 판단 ─────────────────────────────────────────────────────
    # 양손목이 엉덩이보다 아래에 있으면 "내린 상태"
    arms_down = (l_wrist.y > hip_y and r_wrist.y > hip_y)

    # ── 우선순위 분류 (순서 중요) ────────────────────────────────────────────
    if t_pose:
        return "forward"
    if left_arm_up and not right_arm_up:
        return "sit"
    if right_arm_up and not left_arm_up:
        return "stand_up"
    if arms_down:
        return "lie_down"

    return "idle"


def get_arm_angles(lm) -> dict:
    """
    양팔의 어깨→팔꿈치 각도를 계산하여 반환 (디버깅/시각화용).

    Args:
        lm: pose_landmarks.landmark 리스트

    Returns:
        dict: {"left_angle": float, "right_angle": float}
              수평선 기준 각도 (위쪽 양수, 도 단위)
    """
    import numpy as np

    def _angle(shoulder, elbow):
        dx = elbow.x - shoulder.x
        dy = elbow.y - shoulder.y
        return np.degrees(np.arctan2(-dy, dx))  # 위쪽이 양수

    left_angle  = _angle(lm[PoseLM.LEFT_SHOULDER],  lm[PoseLM.LEFT_ELBOW])
    right_angle = _angle(lm[PoseLM.RIGHT_SHOULDER], lm[PoseLM.RIGHT_ELBOW])

    return {"left_angle": left_angle, "right_angle": right_angle}
