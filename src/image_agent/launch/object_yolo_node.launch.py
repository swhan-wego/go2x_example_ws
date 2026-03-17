import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# uv venv의 site-packages를 PYTHONPATH에 추가
_prefix = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)[0]
_workspace = (
    os.path.dirname(os.path.dirname(_prefix)) if _prefix else os.path.expanduser("~")
)
_VENV_SITE = os.path.join(_workspace, ".venv", "lib", "python3.10", "site-packages")
_pythonpath = _VENV_SITE + os.pathsep + os.environ.get("PYTHONPATH", "")


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "image_topic", default_value="/front_camera", description="입력 카메라 토픽"
        ),
        DeclareLaunchArgument(
            "object_topic",
            default_value="/object_image",
            description="검출 결과 이미지 토픽",
        ),
        DeclareLaunchArgument(
            "object_pos_topic",
            default_value="/object_position",
            description="검출 결과 위치 토픽",
        ),
        # 모델 경로: 절대경로 또는 워크스페이스 상대경로
        DeclareLaunchArgument(
            "model_path",
            default_value="models/yolov8n.onnx",
            description="ONNX 모델 경로",
        ),
        # 추적할 COCO 클래스 ID (0=person, 2=car, 15=cat, 16=dog, ...)
        DeclareLaunchArgument(
            "target_class", default_value="0", description="추적할 COCO 클래스 ID"
        ),
        # 검출 신뢰도 임계값
        DeclareLaunchArgument(
            "conf_threshold", default_value="0.4", description="검출 신뢰도 임계값"
        ),
        # NMS IoU 임계값
        DeclareLaunchArgument(
            "iou_threshold", default_value="0.45", description="NMS IoU 임계값"
        ),
        # 모델 입력 해상도
        DeclareLaunchArgument(
            "input_size", default_value="640", description="모델 입력 해상도"
        ),
        # bbox 주변 여백 비율
        DeclareLaunchArgument(
            "padding", default_value="0.05", description="bbox 주변 여백 비율"
        ),
        # True: 검출된 모든 사물 발행 / False: 가장 큰 사물 1개만 발행
        DeclareLaunchArgument(
            "publish_all", default_value="False", description="모든 사물 발행 여부"
        ),
    ]

    node = Node(
        package="image_agent",
        executable="object_yolo_node",
        name="object_yolo_node",
        output="screen",
        additional_env={"PYTHONPATH": _pythonpath},
        parameters=[
            {"image_topic": LaunchConfiguration("image_topic")},
            {"object_topic": LaunchConfiguration("object_topic")},
            {"object_pos_topic": LaunchConfiguration("object_pos_topic")},
            {"model_path": LaunchConfiguration("model_path")},
            {"target_class": LaunchConfiguration("target_class")},
            {"conf_threshold": LaunchConfiguration("conf_threshold")},
            {"iou_threshold": LaunchConfiguration("iou_threshold")},
            {"input_size": LaunchConfiguration("input_size")},
            {"padding": LaunchConfiguration("padding")},
            {"publish_all": LaunchConfiguration("publish_all")},
        ],
    )

    return LaunchDescription(args + [node])


if __name__ == "__main__":
    from launch import LaunchService

    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
