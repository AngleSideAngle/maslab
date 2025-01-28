import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.frontend.parse_substitution import get_package_share_directory
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("vision"), "launch"),
                "/vision_launch.py",
            ]
        )
    )

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("maslab_hardware"), "launch"),
                "/hardware_launch.py",
            ]
        )
    )

    planner_node = Node(
        package="planning",
        executable="planning",
    )

    return LaunchDescription([vision_launch, hardware_launch, planner_node])
