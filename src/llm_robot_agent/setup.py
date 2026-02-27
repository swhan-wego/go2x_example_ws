from setuptools import find_packages, setup
import os
from glob import glob

package_name = "llm_robot_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 런치 파일 설치
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wego",
    maintainer_email="wego@wego-robotics.com",
    description="LLM 연동 ROS2 로봇 에이전트 패키지",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 실행 명령어     = 패키지.모듈:함수
            "llm_interface  = llm_robot_agent.llm_interface_node:main",
            "mcp_server     = llm_robot_agent.mcp_server_node:main",
            "robot_control  = llm_robot_agent.robot_control_node:main",
            "input_publisher= llm_robot_agent.input_publisher:main",
        ],
    },
)
