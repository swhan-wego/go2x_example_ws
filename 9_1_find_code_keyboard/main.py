import cv2
import numpy as np
import asyncio
import threading
import logging
import time
from queue import Queue
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD
from pyzbar.pyzbar import decode

from go2_libs.move_keyboard import WebRtcMoveKeyboard, MoveState

logging.basicConfig(level=logging.FATAL)

# 상태 상수 정의
ST_IDLE = "IDLE"
ST_READY = "READY"
ST_VICTORY = "VICTORY" # 승리 상태 추가


def keyboard_state_handler(new_state : MoveState):
    # 상태 수동 전환
    if new_state is None:   return
    
    if new_state == MoveState.RELEASE: robot_fsm.state = ST_IDLE
    elif new_state == MoveState.READY: robot_fsm.state = ST_READY

conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.0.61")
# conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)
move_keyboard = WebRtcMoveKeyboard(conn, onchange_callback=keyboard_state_handler, is_key_handle=True)

class Go2StateMachine:
    def __init__(self):
        self.state = ST_IDLE
        self.score = 0
        self.target_score = 50 # 50점이 되면 승리!
        self.detected_qrs = set()
        self.frame_queue = Queue()
        self.current_move = {"x": 0.0, "y": 0.0, "z": 0.0}
        
    def reset_score(self):
        self.score = 0
        self.detected_qrs.clear()
        print("[System] 점수와 중복 기록이 초기화되었습니다.")

    def add_qr(self, data):
        if self.state == ST_READY and data not in self.detected_qrs:
            self.detected_qrs.add(data)
            self.score += 10
            print(f"[Score Up!] {data} 인식 | 현재 점수: {self.score}")
            if self.score >= self.target_score:
                self.state = ST_VICTORY
            return True
        return False

robot_fsm = Go2StateMachine()

async def recv_camera_stream(track):
    fail_count = 0
    while True:
        try:
            frame = await track.recv()
            img = frame.to_ndarray(format="bgr24")
            if robot_fsm.frame_queue.qsize() > 1:
                try: robot_fsm.frame_queue.get_nowait()
                except: pass
            robot_fsm.frame_queue.put(img)
            fail_count = 0 # 성공 시 카운트 초기화
        except Exception as e:
            fail_count += 1
            if fail_count > 30: # 약 1초 이상 프레임이 안 오면 종료
                print("[Video] 영상 스트림 유지 실패")
                break
            await asyncio.sleep(0.03) # 잠깐 대기 후 재시도

async def control_logic():
    prev_state = None
    print("[System] 제어 로직 시작")
    
    while True:
        # 상태 전이 로직
        if robot_fsm.state != prev_state:
            if robot_fsm.state == ST_IDLE:
                print("[Mode] 대기 상태: 로봇이 앉습니다.")
                robot_fsm.reset_score()
            
            elif robot_fsm.state == ST_READY:
                print("[Mode] 준비 상태: 로봇이 일어섭니다.")
                await asyncio.sleep(3)  

            elif robot_fsm.state == ST_VICTORY:
                print("[Mode] 승리! 축하 동작을 시작합니다.")
                move_keyboard.hello()
                await asyncio.sleep(5.0) # 동작 완수를 위한 대기
                print("[Mode] 축하 종료. 대기 상태로 복귀합니다.")
                robot_fsm.state = ST_IDLE
            
            prev_state = robot_fsm.state
                
        await asyncio.sleep(0.5)

async def main():
    
    # [수정] WebRTC 연결 및 설정
    await conn.connect()
    conn.video.switchVideoChannel(True)
    conn.video.add_track_callback(recv_camera_stream)
    
    # [핵심] control_logic을 백그라운드 태스크로 직접 실행 (스레드 불필요)
    
    asyncio.create_task(move_keyboard.control_logic())
    asyncio.create_task(control_logic())
    
    cv2.namedWindow("Go2 Quest Mode")
    frame_count = 0
    
    try:
        frame_count = 0
        while True:
            await asyncio.sleep(0.01)
            
            if not robot_fsm.frame_queue.empty():
                img = robot_fsm.frame_queue.get()
                frame_count += 1
                
                # QR 인식 (READY 상태에서만 유효)
                if robot_fsm.state == ST_READY and frame_count % 10 == 0:
                    for qr in decode(img):
                        data = qr.data.decode('utf-8')
                        is_new = robot_fsm.add_qr(data)
                        color = (0, 255, 0) if is_new else (0, 0, 255)
                        pts = np.array(qr.polygon, np.int32).reshape((-1, 1, 2))
                        cv2.polylines(img, [pts], True, color, 3)

                # 상단 UI
                overlay_color = (50, 50, 50) if robot_fsm.state == ST_IDLE else (0, 100, 0)
                if robot_fsm.state == ST_VICTORY: overlay_color = (0, 215, 255)
                
                cv2.rectangle(img, (0, 0), (640, 60), overlay_color, -1)
                status_text = f"[{robot_fsm.state}] SCORE: {robot_fsm.score}/{robot_fsm.target_score}"
                cv2.putText(img, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                
                cv2.imshow("Go2 Quest Mode", img)
                cv2.waitKey(1)

    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(main())