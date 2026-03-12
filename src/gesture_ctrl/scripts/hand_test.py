#!/usr/bin/env python3
"""
hand_test.py — Hand Landmarks 실시간 시각화 테스트 (6.2절)

MediaPipe Hands로 손 21개 랜드마크를 실시간으로 표시하고,
검지 끝(8번) 좌표 및 손가락 상태를 콘솔에 출력합니다.

실행:
    python3 hand_test.py

종료: q 키
"""

import cv2
import mediapipe as mp

from gesture_ctrl.finger_utils import get_finger_states, classify_gesture


mp_hands = mp.solutions.hands
mp_draw  = mp.solutions.drawing_utils


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        return

    print("Hand Landmarks 테스트 시작 — 'q' 키로 종료")

    with mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5,
    ) as hands:

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = hands.process(rgb)

            if result.multi_hand_landmarks and result.multi_handedness:
                hand_lm    = result.multi_hand_landmarks[0]
                handedness = result.multi_handedness[0].classification[0].label

                # 랜드마크 시각화
                mp_draw.draw_landmarks(
                    frame, hand_lm, mp_hands.HAND_CONNECTIONS
                )

                lm = hand_lm.landmark
                h, w, _ = frame.shape

                # 검지 끝(8번) 픽셀 좌표
                tip = lm[8]
                px, py = int(tip.x * w), int(tip.y * h)
                cv2.circle(frame, (px, py), 8, (0, 0, 255), -1)

                # 손가락 상태 및 제스처 분류
                fs      = get_finger_states(lm)
                gesture = classify_gesture(lm, handedness)

                state_str = " ".join(
                    k[0].upper() if v else "_"
                    for k, v in fs.items()
                )
                print(f"[{handedness:5}] 손가락: {state_str}  →  {gesture}")

                # HUD
                cv2.putText(
                    frame, f"{handedness}: {gesture}",
                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 0), 2
                )

            cv2.imshow("Hand Landmarks Test", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()
    print("테스트 종료")


if __name__ == "__main__":
    main()
