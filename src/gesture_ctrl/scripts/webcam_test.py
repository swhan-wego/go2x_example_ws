#!/usr/bin/env python3
"""
webcam_test.py — 웹캠 기본 동작 확인 스크립트 (6.1절)

웹캠이 정상적으로 동작하는지 확인합니다.
화면에 웹캠 영상이 표시되면 준비 완료입니다.

실행:
    python3 webcam_test.py
    python3 webcam_test.py --index 1   # 특정 카메라 인덱스 지정

종료: q 키
"""

import argparse
import cv2


def main():
    parser = argparse.ArgumentParser(description="웹캠 동작 확인")
    parser.add_argument("--index", type=int, default=0, help="카메라 인덱스 번호")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.index)
    if not cap.isOpened():
        print(f"[ERROR] 카메라 {args.index}를 열 수 없습니다.")
        print("        ls /dev/video* 로 연결된 카메라를 확인하세요.")
        return

    width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps    = cap.get(cv2.CAP_PROP_FPS)

    print(f"[OK] 카메라 {args.index} 연결 성공")
    print(f"     해상도: {int(width)} x {int(height)}")
    print(f"     FPS:    {fps:.1f}")
    print("     'q' 키를 눌러 종료하세요.")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] 프레임 읽기 실패")
            break

        # 해상도 정보 HUD
        cv2.putText(
            frame, f"{int(width)}x{int(height)} @ {fps:.0f}fps",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
        )

        cv2.imshow("Webcam Test", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("[OK] 웹캠 테스트 완료")


if __name__ == "__main__":
    main()
