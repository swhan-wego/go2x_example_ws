import asyncio
from enum import Enum
import threading
import sys
import termios
import tty
import time
from typing import Callable
from pynput import keyboard

from flask import json
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

class MoveDirection(Enum):
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4
    TURN_LEFT = 5
    TURN_RIGHT = 6
    STOP = 7
    
class MoveState(Enum):
    RELEASE = "Damp"  # 앉아서 관절에 토크가 없는 상태
    STANDUP = "StandUp"  # 일어서는 중인 상태
    READY = "BalanceStand"   # 일어서서 이동 준비가 완료된 상태
    SITDOWN = "StandDown"  # 앉는 중인 상태

class WebRtcMoveKeyboard:
    def __init__(self, conn : UnitreeWebRTCConnection, onchange_callback : Callable[[MoveState], None] = None, is_key_handle : bool = False):
        self.conn = conn    # WebRTC 연결 객체
        self.move_state = MoveState.RELEASE
        self.current_robot_mode = None  # 로봇의 실제 현재 모드
        self.lock_state = True  # 초기에는 잠금 상태
        self.is_moving = False  # 이동 중 플래그
        
        # 모드 전환 타이머
        self.timeout_task = None # 핸들 대신 Task 객체 저장
        self.mode_change_timeout = 3.0  # 3초
        
        # 이동 속도 설정
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # self.loop = asyncio.new_event_loop()
        self.loop = asyncio.get_event_loop()
        # threading.Thread(target=self.run_asyncio_loop, args=(self.loop,), daemon=True).start()
        self._on_change = onchange_callback
        self.is_key_handle = is_key_handle
        self._pressed_key = ''
        
        if self.is_key_handle:
            print("[i] 키보드 입력 모드 활성화됨.")
            self.listener = keyboard.Listener(on_press=self._on_key_press)
            self.listener.start()
        
    def _on_key_press(self, key):
        try:
            if hasattr(key, 'char') and key.char is not None:
                self._pressed_key = key.char
        except AttributeError:
            pass  # 특수 키는 무시 
        
    async def control_logic(self):
        print("[i] WebRtcMoveKeyboard 제어 로직 시작")

        # 상태 머신 루프
        while True:
            try:
                # 주기적으로 로봇 상태 확인
                await asyncio.wait_for(self._check_robot_state(), timeout=0.5)
                
                if self._pressed_key != '':
                    key = self._pressed_key
                    self._pressed_key = ''
                    await self.default_keyboard_handler(key)
                
                # 상태 머신 로직
                if self.current_robot_mode:
                    # 로봇의 실제 모드와 목표 상태를 비교하여 상태 전환
                    if self.move_state == MoveState.STANDUP:
                        if self.current_robot_mode == "BalanceStand":
                            self._cancel_timeout()
                            self.move_state = MoveState.READY
                            print("[✓] 로봇이 일어났습니다. 이동 준비 완료.")
                            
                    elif self.move_state == MoveState.SITDOWN:
                        if self.current_robot_mode == "Damp":
                            self._cancel_timeout()
                            self.move_state = MoveState.RELEASE
                            print("[✓] 로봇이 앉았습니다. 관절 토크 해제 완료.")

            except asyncio.TimeoutError:
                pass # 상태 확인이 늦어져도 루프는 계속 돎
            except Exception as e:
                print(f"[Loop Error] {e}")
                
            await asyncio.sleep(0.5)
    
    async def stand_up(self):
        """로봇을 일으키는 동작을 수행합니다."""
        if self.move_state == MoveState.RELEASE:
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"],
                {"api_id": SPORT_CMD["RecoveryStand"]}
            )
            self.move_state = MoveState.STANDUP
            self._start_timeout(self._on_standup_timeout)
            print("[→] 로봇이 일어나는 중입니다... (3초 대기)")
            
    async def sit_down(self):
        """로봇을 앉히는 동작을 수행합니다."""
        if self.move_state == MoveState.READY or self.move_state == MoveState.STANDUP:
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"],
                {"api_id": SPORT_CMD["StandDown"]}
            )
            self.move_state = MoveState.SITDOWN
            self._start_timeout(self._on_sitdown_timeout)
            print("[→] 로봇이 앉는 중입니다... (3초 대기)")
            
    async def hello(self):
        """로봇이 해피 모션을 수행합니다."""
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["Hello"]}
        )
        print("[→] 로봇이 해피 모션을 수행합니다.")
            
    async def move(self, direction: MoveDirection):
        """로봇을 이동시킵니다."""
        if self.move_state != MoveState.READY:
            print(f"[!] 로봇이 준비 상태가 아닙니다. 현재 상태: {self.move_state.name}")
            return
            
        vx, vy, vyaw = 0.0, 0.0, 0.0
        
        if direction == MoveDirection.FORWARD:
            vx = self.linear_speed
            print("[↑] 전진")
        elif direction == MoveDirection.BACKWARD:
            vx = -self.linear_speed
            print("[↓] 후진")
        elif direction == MoveDirection.LEFT:
            vy = self.linear_speed
            print("[←] 좌측 이동")
        elif direction == MoveDirection.RIGHT:
            vy = -self.linear_speed
            print("[→] 우측 이동")
        elif direction == MoveDirection.TURN_LEFT:
            vyaw = self.angular_speed
            print("[⟲] 좌회전")
        elif direction == MoveDirection.TURN_RIGHT:
            vyaw = -self.angular_speed
            print("[⟳] 우회전")
        elif direction == MoveDirection.STOP:
            vx, vy, vyaw = 0.0, 0.0, 0.0
            print("[■] 정지")
                        
        print(f"현재 전송 대기 중인 버퍼: {self._check_buffer()} bytes", flush=True)
        if self._check_buffer() > 0:
            print("[!] 이전 명령이 아직 처리 중입니다. 잠시 기다려주세요.")
            return

        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["Move"], "parameter": {"x": vx, "y": vy, "z": vyaw}}
        )
        self.is_moving = (direction != MoveDirection.STOP)
        
    def _check_buffer(self):
        """데이터 채널 버퍼 상태를 확인합니다."""
        current_buffer = getattr(self.conn.datachannel.channel, "_RTCDataChannel__bufferedAmount")
        return current_buffer
            
    def _status_callback(self, msg):
        """로봇에서 상태 메시지가 올 때마다 실행되는 콜백"""
        print(f"[Status Msg] {msg}")
        try:
            # 유니트리 메시지 구조에 따라 데이터 파싱
            if 'data' in msg and 'data' in msg['data']:
                status_data = json.loads(msg['data']['data'])
                self.current_robot_mode = status_data.get('name', 'Unknown')
        except Exception as e:
            print(f"Callback Error: {e}")
    
    async def _check_robot_state(self):
        """로봇의 현재 상태를 확인합니다."""
        try:
            print(f"현재 전송 대기 중인 버퍼: {self._check_buffer()} bytes", flush=True)
            if self._check_buffer() > 0:
                print("[!] 이전 명령이 아직 처리 중입니다. 잠시 기다려주세요.")
                return
            response = await asyncio.wait_for(
                self.conn.datachannel.pub_sub.publish_request_new(
                    RTC_TOPIC["MOTION_SWITCHER"], 
                    {"api_id": 1001}
                ),
                timeout=1.0
            )

            if response['data']['header']['status']['code'] == 0:
                status_data = json.loads(response['data']['data'])
                self.current_robot_mode = status_data.get('name', 'Unknown')
        except asyncio.TimeoutError:
            print("[!] 상태 조회 타임아웃 - 루프 계속 진행")
        except Exception as e:
            pass  # 조용히 처리
            
    def print_status(self):
        """현재 상태를 출력합니다."""
        print("\n" + "="*50)
        print(f"상태 머신: {self.move_state.name}")
        print(f"로봇 모드: {self.current_robot_mode}")
        print(f"이동 중: {'예' if self.is_moving else '아니오'}")
        print("="*50)
    
    def _start_timeout(self, callback):
        """타임아웃 타이머를 시작합니다."""
        self._cancel_timeout()
        # 현재 실행 중인 메인 루프에 타스크 등록
        self.timeout_task = asyncio.create_task(self._timeout_task(callback))

    def _cancel_timeout(self):
        """타임아웃 타이머를 취소합니다."""
        if self.timeout_task:
            self.timeout_task.cancel()
            self.timeout_task = None

    async def _timeout_task(self, callback):
        try:
            await asyncio.sleep(self.mode_change_timeout)
            callback()
        except asyncio.CancelledError:
            pass # 취소 시 정상 종료
        
    def _on_standup_timeout(self):
        """일어서기 타임아웃 콜백입니다."""
        if self.move_state == MoveState.STANDUP:
            self.move_state = MoveState.READY
            print("[✓] 3초 경과 - 로봇이 일어났습니다. 이동 준비 완료.")
            if self._on_change:
                self._on_change(self.move_state)
    
    def _on_sitdown_timeout(self):
        """앉기 타임아웃 콜백입니다."""
        if self.move_state == MoveState.SITDOWN:
            self.move_state = MoveState.RELEASE
            print("[✓] 3초 경과 - 로봇이 앉았습니다. 관절 토크 해제 완료.")
            if self._on_change:
                self._on_change(self.move_state)

    # async def waiting_for_stand_ready(self):
    #     """로봇이 일어서서 이동 준비가 완료될 때까지 대기합니다."""
    #     while self.move_state != MoveState.READY:
    #         await asyncio.sleep(0.1)
            
    async def _get_state(self):
        """로봇의 현재 상태를 가져옵니다 (디버깅용)."""
        try:
            print(f"현재 전송 대기 중인 버퍼: {self._check_buffer()} bytes", flush=True)
            if self._check_buffer() > 0:
                print("[!] 이전 명령이 아직 처리 중입니다. 잠시 기다려주세요.")
                return
            
            response = await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"], 
                {"api_id": 1001}
            )

            if response['data']['header']['status']['code'] == 0:
                status_data = json.loads(response['data']['data'])
                current_mode = status_data['name']
                
                print("--------------------------------")
                print(f"✅ 현재 로봇 모드: {current_mode}")
                print("--------------------------------")
                return current_mode
            else:
                print("❌ 상태 조회 실패")
                return None

        except Exception as e:
            print(f"에러 발생: {e}")
            return None
        
    def print_default_keyboard(self):
        print(" [ 모드 및 상태 ]               |  [ 이동 제어 (Standing) ]")
        print("-" * 66)
        print("[1] 일어서기 (Stand Up)        |  [w] 전진    [q] 좌회전")
        print("[2] 앉기    (Sit Down)         |  [s] 후진    [e] 우회전")
        print("[i] 상태 출력 (Info)           |  [a] 좌측    [d] 우측")
        print("[ESC] 프로그램 종료            |  [Space] 정지 (Stop)")
        print("="*66)
        
    async def default_keyboard_handler(self, key):
        # 모드 변경 명령
        if key == '1':
            await self.stand_up()
        elif key == '2':
            await self.sit_down()
            
        # 이동 명령
        elif key == 'w':
            await self.move(MoveDirection.FORWARD)
        elif key == 's':
            await self.move(MoveDirection.BACKWARD)
        elif key == 'a':
            await self.move(MoveDirection.LEFT)
        elif key == 'd':
            await self.move(MoveDirection.RIGHT)
        elif key == 'q':
            await self.move(MoveDirection.TURN_LEFT)
        elif key == 'e':
            await self.move(MoveDirection.TURN_RIGHT)
        elif key == ' ':
            await self.move(MoveDirection.STOP)
            
        # 상태 출력
        elif key == 'i':
            self.print_status()
            
async def main():
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)
    await conn.connect()
    conn.video.switchVideoChannel(False)
    mv_key = WebRtcMoveKeyboard(conn)

    # 초기화 대기
    import time
    time.sleep(3)
    
    mv_key.print_default_keyboard()
    
    try:
        while True:
            # 키보드 입력 핸들러 방식으로 추가
            
            await asyncio.sleep(1)                
    except KeyboardInterrupt:
        print("\n프로그램 종료")   

if __name__ == "__main__":
    asyncio.run(main())