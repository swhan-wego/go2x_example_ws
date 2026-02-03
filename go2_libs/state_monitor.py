import asyncio
from enum import Enum
import threading
import sys

from flask import json
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

class stateMonitor:
    def __init__(self, conn : UnitreeWebRTCConnection):
        self.conn = conn 
        self.robot_state = None

        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.run_asyncio_loop, args=(self.loop,), daemon=True).start()
        pass
    
    def run_asyncio_loop(self, loop):
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.monitor_logic())
        
    async def monitor_logic(self):
        await self.conn.connect()
        self.conn.video.switchVideoChannel(False)
        
        self.conn.datachannel.pub_sub.subscribe(
            # RTC_TOPIC["LOW_STATE"], 
            "rt/api/motion_switcher/response",
            self._status_callback
        )
        
        while True:
            # 여기서 실시간으로 업데이트되는 self.robot_state를 활용해 로직 결정
            if self.robot_state:
                # 예: 로봇의 현재 z축 높이가 일정 수준 이상이면 일어난 것으로 간주
                # print(f"Current Height: {self.robot_state.get('position', [0,0,0])[2]}")
                pass
            sys.stdout.write("\033[H\033[J")
            pretty_json = json.dumps(self.robot_state, indent=4, sort_keys=True)
            print(f"{pretty_json}")
            
            
            await asyncio.sleep(1) # 구독 방식이므로 루프 주기를 짧게 가져가도 됩니다.        
        
        # while True:
        #     response = await self.conn.datachannel.pub_sub.publish_request_new(
        #         RTC_TOPIC["LOW_STATE"], 
        #         {"api_id": 1001}
        #     )
        #     if response is not None:
        #         print(response)
        #         # self.state = json.loads(response)
        #         # print(self.state)
        #     await asyncio.sleep(0.01)
        
    def _status_callback(self, msg):
        """로봇에서 상태 메시지가 올 때마다 실행되는 콜백"""
        try:
            # 유니트리 메시지 구조에 따라 데이터 파싱
            # 보통 msg['data'] 또는 msg 자체에 상태 값이 담깁니다.
            self.robot_state = msg 
        except Exception as e:
            print(f"Callback Error: {e}")
         
def main():
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)
    state_monitor = stateMonitor(conn)

    # 메인 스레드는 다른 작업을 수행할 수 있습니다.
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("프로그램 종료")   

if __name__ == "__main__":
    main()