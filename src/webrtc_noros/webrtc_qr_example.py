"""QR Code 감지 예제

간단 사용법:
    python webrtc_qr_example.py

기능:
  - WebRTC를 통해 Go2 로봇의 카메라 스트림 수신
  - 실시간 QR 코드 감지 및 디코딩
  - 3가지 감지 방식 지원 (OpenCV, PyZBar, WeChat)

조작:
    - 'q' 키: 프로그램 종료
"""

import cv2
import numpy as np
import asyncio
import logging
import threading
import time
from queue import Queue
from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)
from unitree_webrtc_connect.constants import RTC_TOPIC
from aiortc import MediaStreamTrack
from pyzbar.pyzbar import decode

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)

detector = cv2.wechat_qrcode_WeChatQRCode(
    "aseset/detect_2021nov.prototxt",
    "aseset/detect_2021nov.caffemodel",
    "aseset/sr_2021nov.prototxt",
    "aseset/sr_2021nov.caffemodel",
)


def main():
    frame_queue = Queue()

    # Choose a connection method (uncomment the correct one)
    # conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.8.181")
    # conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, serialNumber="B42D2000XXXXXXXX")
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)

    async def turnon_led():
        # Turn on LED flash
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["VUI"], {"api_id": 1005, "parameter": {"brightness": 10}}
        )
        await asyncio.sleep(0.5)

    async def turnoff_led():
        # Turn on LED flash
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["VUI"], {"api_id": 1005, "parameter": {"brightness": 0}}
        )
        await asyncio.sleep(0.5)

    def draw_qr_result(img, data, points, color=(0, 255, 0)):
        # 1. 좌표 데이터 전처리 (항상 2차원 리스트/배열 형태로 변환)
        pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))

        # 2. 다각형(사각형) 그리기
        cv2.polylines(img, [pts], isClosed=True, color=color, thickness=3)

        # 3. 텍스트 출력 (첫 번째 좌표 기준 위쪽)
        x, y = pts[0][0]
        cv2.putText(
            img, data, (int(x), int(y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
        )

    def qr_detector_cv(img):
        data, bbox, _ = qr_decoder.detectAndDecode(img)
        if data and bbox is not None:
            print(f"[CV default]: {data}")
            # CV Default는 보통 (1, 4, 2) 형태이므로 첫 번째 요소를 전달
            draw_qr_result(img, data, bbox[0], color=(0, 255, 0))

    def qr_detector_pyzbar(img):
        from pyzbar.pyzbar import decode

        for qr in decode(img):
            data = qr.data.decode("utf-8")
            print(f"[PYZBAR]: {data}")
            # pyzbar의 polygon 좌표를 바로 전달
            draw_qr_result(img, data, qr.polygon, color=(255, 0, 255))

    def qr_detector_wechat(img):
        res, points = detector.detectAndDecode(img)
        for i, data in enumerate(res):
            if data:
                print(f"[WECHAT]: {data}")
                # WeChat은 리스트 안에 여러 QR 좌표가 있으므로 인덱스로 전달
                draw_qr_result(img, data, points[i], color=(255, 255, 0))

    # Async function to receive video frames and put them in the queue
    async def recv_camera_stream(track: MediaStreamTrack):
        while True:
            frame = await track.recv()
            # Convert the frame to a NumPy array
            img = frame.to_ndarray(format="bgr24")
            frame_queue.put(img)

    def run_asyncio_loop(loop):
        asyncio.set_event_loop(loop)

        async def setup():
            try:
                # Connect to the device
                await conn.connect()

                # Turn on LED flash
                await turnoff_led()
                await turnon_led()

                # Switch video channel on and start receiving video frames
                conn.video.switchVideoChannel(True)

                # Add callback to handle received video frames
                conn.video.add_track_callback(recv_camera_stream)

            except Exception as e:
                logging.error(f"Error in WebRTC connection: {e}")

        # Run the setup coroutine and then start the event loop
        loop.run_until_complete(setup())
        loop.run_forever()

    # Create a new event loop for the asyncio code
    loop = asyncio.new_event_loop()

    # Start the asyncio event loop in a separate thread
    asyncio_thread = threading.Thread(target=run_asyncio_loop, args=(loop,))
    asyncio_thread.start()

    qr_decoder = cv2.QRCodeDetector()

    try:
        while True:
            if not frame_queue.empty():
                img = frame_queue.get()

                # qr_detector_cv(img)
                qr_detector_pyzbar(img)
                qr_detector_wechat(img)

                cv2.imshow("Go2 WebRTC Stream + QR", img)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:
                # Sleep briefly to prevent high CPU usage
                time.sleep(0.01)
    finally:
        cv2.destroyAllWindows()
        # Stop the asyncio event loop
        loop.call_soon_threadsafe(loop.stop)
        asyncio_thread.join()


if __name__ == "__main__":
    main()
