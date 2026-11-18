from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='watertank_utils',
            executable='publish_frame.py',
            name='broadcaster1',
        ),
    ])
