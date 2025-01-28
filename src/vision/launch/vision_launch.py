import os

from launch import LaunchDescription
from launch.frontend.parse_substitution import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # hardware_config = os.path.join(
    #     get_package_share_directory("maslab_hardware"), "config", "params.yaml"
    # )

    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        parameters=[{"image_size": [640, 480]}],  # TODO
    )

    cube_detect_node = Node(
        package="vision",
        executable="cube_detect",
        name="cube_detect",
    )

    return LaunchDescription([camera_node, cube_detect_node])
