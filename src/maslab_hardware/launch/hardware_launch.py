from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hardware_node = Node(
        package="maslab_hardware",
        executable="maslab_robot",
        name="robot",
        emulate_tty=True,
        # parameters=[{"p_gain", "int(1)"}],
    )

    return LaunchDescription([hardware_node])
