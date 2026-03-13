#!/usr/bin/env python3
"""YOLOv8n ONNX 모델 다운로드 스크립트.

ultralytics를 이용해 yolov8n.pt를 다운로드하고 ONNX로 변환합니다.
변환된 모델은 <workspace>/models/yolov8n.onnx 에 저장됩니다.

사용법:
  cd <workspace>
  pip install ultralytics   # 최초 1회
  python3 src/image_agent/scripts/download_yolo_model.py
"""

import os
import shutil
import sys

# 워크스페이스 루트 = 이 스크립트 기준으로 4단계 위
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_WS_ROOT = os.path.abspath(os.path.join(_SCRIPT_DIR, "..", "..", ".."))
_OUT_DIR  = os.path.join(_WS_ROOT, "models")
_OUT_PATH = os.path.join(_OUT_DIR, "yolov8n.onnx")


def main():
    if os.path.isfile(_OUT_PATH):
        print(f"이미 존재합니다: {_OUT_PATH}")
        return

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ultralytics가 설치되어 있지 않습니다.")
        print("  pip install ultralytics")
        sys.exit(1)

    print("yolov8n.pt 다운로드 및 ONNX 변환 중...")
    model = YOLO("yolov8n.pt")
    export_path = model.export(format="onnx", imgsz=640, opset=12, simplify=True)

    os.makedirs(_OUT_DIR, exist_ok=True)
    shutil.copy(str(export_path), _OUT_PATH)
    print(f"저장 완료: {_OUT_PATH}")


if __name__ == "__main__":
    main()
