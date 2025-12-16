from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gripper_node = Node(
        package="robotiq2f",
        executable="robotiqhande_service",
        name="robotiq_hande_node",
        output="screen",
        parameters=[
            {"comport": "/dev/ttyUSB0"},
            {"baud": 115200},
            {"use_fake_hardware": False},
        ],
    )

    return LaunchDescription([gripper_node])
