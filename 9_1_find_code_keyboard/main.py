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

logging.basicConfig(level=logging.FATAL)

# 상태 상수 정의
ST_IDLE = "IDLE"
ST_READY = "READY"
ST_VICTORY = "VICTORY" # 승리 상태 추가

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

def main():
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)

    async def recv_camera_stream(track):
        while True:
            try:
                frame = await track.recv()
                img = frame.to_ndarray(format="bgr24")
                if robot_fsm.frame_queue.qsize() > 1:
                    try: robot_fsm.frame_queue.get_nowait()
                    except: pass
                robot_fsm.frame_queue.put(img)
            except: break

    def run_asyncio_loop(loop):
        asyncio.set_event_loop(loop)
        async def control_logic():
            await conn.connect()
            conn.video.switchVideoChannel(True)
            conn.video.add_track_callback(recv_camera_stream)
            
            prev_state = None

            while True:
                # 상태 전이 로직
                if robot_fsm.state != prev_state:
                    if robot_fsm.state == ST_IDLE:
                        print("[Mode] 대기 상태: 로봇이 앉습니다.")
                        await conn.datachannel.pub_sub.publish_request_new(
                            # 로봇이 앉는 동작 호출 SPORT_CMD 사용
                            RTC_TOPIC["SPORT_MOD"],
                            {"api_id": SPORT_CMD["StandDown"]}
                        )
                        robot_fsm.reset_score()
                    
                    elif robot_fsm.state == ST_READY:
                        print("[Mode] 준비 상태: 로봇이 일어섭니다.")
                        await conn.datachannel.pub_sub.publish_request_new(
                            # 로봇이 일어나는 동작 호출 SPORT_CMD 사용
                            RTC_TOPIC["SPORT_MOD"], 
                            {"api_id": SPORT_CMD["RecoveryStand"]}
                        )
                        await asyncio.sleep(3)

                    elif robot_fsm.state == ST_VICTORY:
                        print("[Mode] 승리! 축하 동작을 시작합니다.")
                        # 1. 하트 그리기 또는 춤 (예: Hello -> Stretch -> Happy)
                        for action in ["Hello", "Happy", "Stretch"]:
                            await conn.datachannel.pub_sub.publish_request_new(
                                RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD[action]}
                            )
                            await asyncio.sleep(3.0) # 동작 완수를 위한 대기
                        
                        print("[Mode] 축하 종료. 대기 상태로 복귀합니다.")
                        robot_fsm.state = ST_IDLE
                    
                    prev_state = robot_fsm.state

                # 이동 명령 (READY일 때만 전송)
                if robot_fsm.state == ST_READY:
                    await conn.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["Move"], "parameter": robot_fsm.current_move}
                    )
                
                await asyncio.sleep(0.1)

        loop.run_until_complete(control_logic())

    loop = asyncio.new_event_loop()
    threading.Thread(target=run_asyncio_loop, args=(loop,), daemon=True).start()

    cv2.namedWindow("Go2 Quest Mode")
    
    try:
        while True:
            if not robot_fsm.frame_queue.empty():
                img = robot_fsm.frame_queue.get()
                
                # QR 인식 (READY 상태에서만 유효)
                if robot_fsm.state == ST_READY:
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

            key = cv2.waitKey(30) & 0xFF
            if key == 27: break
            
            # 상태 수동 전환
            if key == ord('1'): robot_fsm.state = ST_IDLE
            if key == ord('2'): robot_fsm.state = ST_READY

            # 조종 (READY일 때만 동작)
            vx, vy, vz = 0.0, 0.0, 0.0
            if robot_fsm.state == ST_READY:
                if key == ord('w'): vx = 0.8
                elif key == ord('s'): vx = -0.8
                elif key == ord('a'): vz = 0.6
                elif key == ord('d'): vz = -0.6
            
            robot_fsm.current_move = {"x": vx, "y": vy, "z": vz}

    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()