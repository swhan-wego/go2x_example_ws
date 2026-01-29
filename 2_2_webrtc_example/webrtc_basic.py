import asyncio
import json
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC

async def main():
    # 1. 로봇 연결 (LocalAP: 로봇 와이파이 연결 시)
    # conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.8.181")
    # conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, serialNumber="B42D2000XXXXXXXX")
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)
    
    print("로봇에 연결 중...")
    await conn.connect()
    print("연결 성공!")

    try:
        # 2. 상태 조회 명령 전송
        # Topic: Motion Switcher (모드 제어기)
        # API ID 1001: "현재 모드 알려줘" (Get Mode)
        response = await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["MOTION_SWITCHER"], 
            {"api_id": 1001}
        )

        # 3. 응답 데이터 파싱
        # 반환값 구조: response['data']['data'] 안에 JSON 문자열이 들어있음
        if response['data']['header']['status']['code'] == 0:
            status_data = json.loads(response['data']['data'])
            current_mode = status_data['name']
            
            print("--------------------------------")
            print(f"✅ 현재 로봇 모드: {current_mode}")
            print("--------------------------------")
        else:
            print("❌ 상태 조회 실패")

    except Exception as e:
        print(f"에러 발생: {e}")

    finally:
        # 연결 종료
        await conn.disconnect()

if __name__ == "__main__":
    asyncio.run(main())