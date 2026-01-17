from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="bt_demo",
                executable="bt_executor_node",
                name="bt_executor",
                output="screen",
                parameters=[
                    {"tick_hz": 10.0},
                    {"low_battery_threshold": 0.2},
                ],
            )
        ]
    )

