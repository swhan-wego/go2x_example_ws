"""
llm_robot_agent.launch.py
─────────────────────────────────────────────────────────────
전체 LLM 로봇 에이전트 시스템을 한 번에 실행하는 런치 파일.

실행 노드:
  1. llm_interface_node  — Ollama LLM 호출 및 Tool Call 파싱
  2. mcp_server_node     — Tool 실행 중계 및 안전 검증
  3. robot_control_node  — Twist 생성 및 /cmd_vel 발행

사용 예시:
  # 기본 실행
  ros2 launch llm_robot_agent llm_robot_agent.launch.py

  # 모델 변경
  ros2 launch llm_robot_agent llm_robot_agent.launch.py model_name:=llama3.2:3b

  # 최대 속도 변경 (안전 테스트 시 낮게 설정)
  ros2 launch llm_robot_agent llm_robot_agent.launch.py max_speed:=0.10
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ── 런치 인자 선언 ────────────────────────────────────────────────────────
    model_arg = DeclareLaunchArgument(
        "model_name",
        default_value="llama3.1:8b",
        description="Ollama 모델 이름 (예: llama3.1:8b, llama3.2:3b)",
    )
    ollama_host_arg = DeclareLaunchArgument(
        "ollama_host",
        default_value="http://localhost:11434",
        description="Ollama 서버 주소",
    )
    max_speed_arg = DeclareLaunchArgument(
        "max_speed",
        default_value="0.22",
        description="로봇 최대 선속도 (m/s) — 안전 테스트 시 0.10 권장",
    )
    max_duration_arg = DeclareLaunchArgument(
        "max_duration",
        default_value="10.0",
        description="최대 동작 지속 시간 (초)",
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="10.0",
        description="cmd_vel 발행 주파수 (Hz)",
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="로봇 제어 토픽 이름",
    )

    # ── 노드 정의 ─────────────────────────────────────────────────────────────

    # 1. LLM 인터페이스 노드
    llm_interface_node = Node(
        package="llm_robot_agent",
        executable="llm_interface",
        name="llm_interface_node",
        parameters=[{
            "model_name":  LaunchConfiguration("model_name"),
            "ollama_host": LaunchConfiguration("ollama_host"),
            "max_history": 10,
        }],
        output="screen",
        emulate_tty=True,
    )

    # 2. MCP 서버 노드
    mcp_server_node = Node(
        package="llm_robot_agent",
        executable="mcp_server",
        name="mcp_server_node",
        parameters=[{
            "max_speed":    LaunchConfiguration("max_speed"),
            "max_duration": LaunchConfiguration("max_duration"),
        }],
        output="screen",
        emulate_tty=True,
    )

    # 3. 로봇 제어 노드
    robot_control_node = Node(
        package="llm_robot_agent",
        executable="robot_control",
        name="robot_control_node",
        parameters=[{
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "publish_rate":  LaunchConfiguration("publish_rate"),
        }],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        # 런치 인자
        model_arg,
        ollama_host_arg,
        max_speed_arg,
        max_duration_arg,
        publish_rate_arg,
        cmd_vel_topic_arg,
        # 시작 메시지
        LogInfo(msg="=== LLM 로봇 에이전트 시스템 시작 ==="),
        # 노드 실행
        llm_interface_node,
        mcp_server_node,
        robot_control_node,
    ])
