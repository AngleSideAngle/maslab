import os

from launch import LaunchDescription
from launch.frontend.parse_substitution import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    hardware_config = os.path.join(
        get_package_share_directory("maslab_hardware"), "config", "params.yaml"
    )

    hardware_node = Node(
        package="maslab_hardware",
        executable="maslab_robot",
        name="robot",
        emulate_tty=True,
        parameters=[hardware_config],
    )

    return LaunchDescription([hardware_node])
