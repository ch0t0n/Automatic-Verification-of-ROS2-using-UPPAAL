from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_model_example',
            executable='publisher1',
        ),
        Node(
            package='ros_model_example',
            executable='subscriber1',
        ),
        Node(
            package='ros_model_example',
            executable='subscriber2',
        ),
    ])
