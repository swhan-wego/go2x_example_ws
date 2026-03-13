from setuptools import find_packages, setup

package_name = 'image_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/rtsp_image_node.launch.py',
            'launch/person_crop_node.launch.py',
            'launch/person_yolo_node.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wego',
    maintainer_email='sw.han@wego-robotics.com',
    description='RTSP 스트림에서 영상을 받아 ROS2 이미지 토픽으로 퍼블리시하는 패키지',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rtsp_image_node = image_agent.rtsp_image_node:main',
            'person_crop_node = image_agent.person_crop_node:main',
            'person_yolo_node = image_agent.person_yolo_node:main',
        ],
    },
)
