import rclpy
from rclpy.node import Node
import time
import sys

# Unitree SDK2 관련 임포트
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

class Go2AdvancedMove(Node):
    def __init__(self, interface_name):
        super().__init__('go2_advanced_move')
        
        # 1. SDK 초기화 (제공하신 예제 방식 적용)
        self.get_logger().info(f"Initializing SDK on interface: {interface_name}")
        ChannelFactoryInitialize(0, interface_name)
        
        # 2. SportClient 설정 및 초기화
        self.client = SportClient()
        self.client.SetTimeout(10.0)
        self.client.Init() # 예제 코드의 Init() 호출 추가
        
        # 초기 상태 설정
        self.start_time = time.time()
        self.state = "STAND_UP"
        
        # 0.1초마다 실행되는 제어 루프
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Go2 제어 노드가 시작되었습니다.")

    def control_loop(self):
        elapsed_time = time.time() - self.start_time

        # 상태 기반 로직 (FSM)
        if self.state == "STAND_UP":
            # 기립 명령 수행
            self.client.StandUp()
            if elapsed_time > 3.0: # 2초 후 다음 단계로
                # 이동 모드로 확실히 진입하기 위해 RecoveryStand 호출
                self.client.RecoveryStand()
                self.state = "FORWARD"
                self.get_logger().info("상태 전환: 전진")

        elif self.state == "FORWARD":
            if elapsed_time < 5.0: # 2초~5초 (3초간 전진)
                self.client.Move(0.3, 0.0, 0.0)
            else:
                self.state = "LATERAL"
                self.get_logger().info("상태 전환: 후진")

        elif self.state == "LATERAL":
            if elapsed_time < 8.0: # 5초~8초 (3초간 옆으로)
                self.client.Move(-0.3, 0.0, 0.0)
            else:
                self.state = "STOP"
                self.get_logger().info("상태 전환: 정지 및 종료")

        elif self.state == "STOP":
            self.client.StopMove()
            self.get_logger().info("모든 동작이 완료되었습니다.")
            self.timer.cancel()
            # 필요 시 여기서 rclpy.shutdown() 호출 가능

def main():
    # 실행 시 인터페이스 이름을 인자로 받음 (예: eth0, wlan0)
    interface = sys.argv[1] if len(sys.argv) > 1 else "eth0"

    print("경고: 로봇 주변에 장애물이 없는지 확인하십시오.")
    time.sleep(1)

    rclpy.init()
    node = Go2AdvancedMove(interface)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n사용자 종료 요청")
    finally:
        # 종료 시 안전을 위해 댐핑 상태로 전환하거나 정지
        if hasattr(node, 'client'):
            node.client.StopMove()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()