from setuptools import find_packages, setup

package_name = "webrtc_app"

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
                "launch/qrgame_launch.py",
                "launch/color_track_basic.py",
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
            "webrtc_qrgame = webrtc_app.webrtc_ros_qrgame:main",
            "webrtc_yologame = webrtc_app.webrtc_ros_yologame:main",
            "color_block_tracker = webrtc_app.color_block_tracker:main",
            "color_block_visualizer = webrtc_app.color_block_visualizer:main",
        ],
    },
)
