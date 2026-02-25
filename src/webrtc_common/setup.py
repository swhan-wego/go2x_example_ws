from setuptools import find_packages, setup

package_name = "webrtc_common"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/unitree_launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/key_config.yaml",
                "config/unitree_params.yaml",
            ],
        ),
        (
            "share/" + package_name + "/models",
            [
                "models/detect_2021nov.prototxt",
                "models/detect_2021nov.caffemodel",
                "models/sr_2021nov.prototxt",
                "models/sr_2021nov.caffemodel",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wego",
    maintainer_email="sw.han@wego-robotics.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "webrtc_unitree_ros = webrtc_common.webrtc_unitree_ros:main",
            "webrtc_keyboard_handler = webrtc_common.keyboard_handler:main",
            "webrtc_terminal_keyboard = webrtc_common.terminal_keyboard:main",
            "webrtc_qrcode = webrtc_common.qrcode:main",
            "webrtc_webcam = webrtc_common.webcam:main",
        ],
    },
)
