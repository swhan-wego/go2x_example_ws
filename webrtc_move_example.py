import cv2
import numpy as np
import asyncio
import logging
import threading
import time
import json
from queue import Queue

# Unitree WebRTC 라이브러리
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

# 로그 설정
logging.basicConfig(level=logging.FATAL)

# 이동 속도 설정
MOVE_SPEED = 0.5   # 전진/후진 (m/s)
SIDE_SPEED = 0.3   # 좌우 게걸음 (m/s)
TURN_SPEED = 1.0   # 회전 (rad/s)

def main():
    command_queue = Queue()
    frame_queue = Queue()

    # 1. 연결 설정 (환경에 맞게 주석 해제)
    # Choose a connection method (uncomment the correct one)
    # conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.8.181")
    # conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, serialNumber="B42D2000XXXXXXXX")
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)

    # -----------------------------------------------------------
    # [비동기] 비디오 스트림 수신
    # -----------------------------------------------------------
    async def recv_camera_stream(track):
        while True:
            try:
                frame = await track.recv()
                img = frame.to_ndarray(format="bgr24")
                if frame_queue.qsize() > 2: # 큐가 꽉 차면 비우기 (지연 방지)
                    frame_queue.get()
                frame_queue.put(img)
            except Exception:
                break

    # -----------------------------------------------------------
    # [비동기] 로봇 제어 루프 (모드 변경 + 명령 전송)
    # -----------------------------------------------------------
    def run_asyncio_loop(loop, cmd_queue):
        asyncio.set_event_loop(loop)

        async def setup_and_control():
            print("[System] 로봇 연결 시도 중...")
            await conn.connect()
            print("[System] 연결 성공!")

            # 1. 모드 확인 및 'normal' 모드로 전환 (핵심!)
            try:
                print("[System] 현재 모션 모드 확인 중...")
                # 현재 모드 조회 (API 1001)
                response = await conn.datachannel.pub_sub.publish_request_new(
                    RTC_TOPIC["MOTION_SWITCHER"], 
                    {"api_id": 1001}
                )
                
                current_mode = "unknown"
                if response['data']['header']['status']['code'] == 0:
                    data = json.loads(response['data']['data'])
                    current_mode = data['name']
                    print(f"[System] 현재 모드: {current_mode}")

                # Normal 모드가 아니라면 전환
                if current_mode != "normal":
                    print(f"[System] 'normal' 모드로 전환합니다... (잠시 대기)")
                    await conn.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["MOTION_SWITCHER"], 
                        {
                            "api_id": 1002,
                            "parameter": {"name": "normal"}
                        }
                    )
                    await asyncio.sleep(5) # 일어서는 시간 대기
                    print("[System] 전환 완료. 제어 준비 됨.")
                
            except Exception as e:
                print(f"[Error] 모드 전환 중 오류: {e}")

            # 2. 비디오 채널 켜기
            conn.video.switchVideoChannel(True)
            conn.video.add_track_callback(recv_camera_stream)

            # 3. 명령 처리 루프
            print("[Ready] 키보드 제어 시작 (W/A/S/D)")
            while True:
                if not cmd_queue.empty():
                    cmd = cmd_queue.get()
                    print("Command In", cmd["type"])
                    # 이동 명령 처리
                    if cmd["type"] == "move":
                        # 참고 코드에 따라 RTC_TOPIC["SPORT_MOD"] 사용
                        await conn.datachannel.pub_sub.publish_request_new(
                            RTC_TOPIC["SPORT_MOD"], 
                            {
                                "api_id": SPORT_CMD["Move"], # API ID: 1008
                                "parameter": {"x": cmd["x"], "y": cmd["y"], "z": cmd["z"]}
                            }
                        )
                    elif cmd["type"] == "balance_stand":
                        await conn.datachannel.pub_sub.publish_request_new(
                            RTC_TOPIC["SPORT_MOD"], 
                            {"api_id": 1002} # 1002: RecoveryStand (일어서서 균형 잡기)
                        )
                    # 특수 동작 (예: 인사하기, 엎드리기 등 필요 시 추가)
                    elif cmd["type"] == "hello":
                        await conn.datachannel.pub_sub.publish_request_new(
                            RTC_TOPIC["SPORT_MOD"], 
                            {"api_id": SPORT_CMD["Hello"]}
                        )

                # CPU 과부하 방지 및 명령 주기 조절
                await asyncio.sleep(0.05)

        # 루프 실행
        loop.run_until_complete(setup_and_control())
        loop.run_forever()

    # -----------------------------------------------------------
    # [메인] 스레드 시작 및 키보드 이벤트 처리
    # -----------------------------------------------------------
    loop = asyncio.new_event_loop()
    asyncio_thread = threading.Thread(target=run_asyncio_loop, args=(loop, command_queue))
    asyncio_thread.start()

    print("=============================================")
    print("      Unitree Go2 Keyboard Controller        ")
    print("---------------------------------------------")
    print(" [이동] W:전진 S:후진 A:좌회전 D:우회전")
    print(" [이동] Q:좌이동 E:우이동 (게걸음)")
    print(" [액션] H:인사하기 (Hello)")
    print(" [종료] ESC")
    print("=============================================")
    
    img = img = np.zeros((480, 640, 3), dtype=np.uint8)
    try:
        vx, vy, vz = 0.0, 0.0, 0.0
        
        while True:
            # 영상 출력 (없으면 검은 화면)
            if not frame_queue.empty():
                img = frame_queue.get()

            # 상태 텍스트
            info = f"CMD: x={vx:.1f}, y={vy:.1f}, z={vz:.1f}"
            cv2.putText(img, info, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Go2 Control", img)

            # 키 입력 처리 (Wait 50ms)
            key = cv2.waitKey(1) & 0xFF

            # 키보드 로직: 누를 때만 속도 설정, 떼면 0으로 초기화
            vx, vy, vz = 0.0, 0.0, 0.0

            if key == 27: # ESC
                break
            elif key == ord('w'): vx = MOVE_SPEED
            elif key == ord('s'): vx = -MOVE_SPEED
            elif key == ord('a'): vz = TURN_SPEED
            elif key == ord('d'): vz = -TURN_SPEED
            elif key == ord('q'): vy = SIDE_SPEED
            elif key == ord('e'): vy = -SIDE_SPEED
            elif key == ord('h'): # H 키 누르면 인사
                command_queue.put({"type": "hello"})
                continue # 인사는 이동 명령과 겹치지 않게
            elif key == 13: 
                command_queue.put({"type": "balance_stand"})
                continue # 이동 명령과 겹치지 않도록 건너뜀
            
            if (vx != 0.0 or vy != 0.0 or vz != 0.0):
                # 큐에 이동 명령 전송 (매 프레임 보냄)
                command_queue.put({"type": "move", "x": vx, "y": vy, "z": vz})

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        loop.call_soon_threadsafe(loop.stop)
        asyncio_thread.join()

if __name__ == "__main__":
    main()