#!/usr/bin/env python3
"""
pose_test.py — Pose Landmarks 실시간 시각화 테스트 (6.2절)

MediaPipe Pose로 전신 33개 랜드마크를 실시간으로 표시하고,
팔 올리기 상태 및 포즈 분류 결과를 콘솔에 출력합니다.

실행:
    python3 pose_test.py

종료: q 키
"""

import cv2
import mediapipe as mp

from gesture_ctrl.pose_utils import classify_pose, get_arm_angles


mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        return

    print("Pose Landmarks 테스트 시작 — 'q' 키로 종료")

    with mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5,
    ) as pose:

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = pose.process(rgb)

            if result.pose_landmarks:
                mp_draw.draw_landmarks(
                    frame,
                    result.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS
                )

                lm      = result.pose_landmarks.landmark
                gesture = classify_pose(lm)
                angles  = get_arm_angles(lm)

                print(
                    f"자세: {gesture:12}  "
                    f"왼팔: {angles['left_angle']:+6.1f}°  "
                    f"오른팔: {angles['right_angle']:+6.1f}°"
                )

                color = (0, 200, 0) if gesture != "idle" else (100, 100, 100)
                cv2.putText(
                    frame, f"Pose: {gesture}",
                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2
                )
                cv2.putText(
                    frame,
                    f"L:{angles['left_angle']:+.0f}  R:{angles['right_angle']:+.0f}",
                    (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 0), 2
                )

            cv2.imshow("Pose Landmarks Test", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()
    print("테스트 종료")


if __name__ == "__main__":
    main()
