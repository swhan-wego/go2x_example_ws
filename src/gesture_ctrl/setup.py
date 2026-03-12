from setuptools import setup, find_packages
import os
from glob import glob

package_name = "gesture_ctrl"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament 인덱스 등록
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        # package.xml
        ("share/" + package_name, ["package.xml"]),
        # 런치 파일
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wego",
    maintainer_email="wego@example.com",
    description="6장: MediaPipe 기반 손/포즈 제스처로 Go2 로봇 제어",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gesture_node = gesture_ctrl.gesture_node:main",
        ],
    },
)
