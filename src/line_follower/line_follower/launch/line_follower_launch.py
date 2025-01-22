from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_follower',
            executable='line_follower',
            name='line_follower',
            parameters=[
                {'speed_drive': 0.1},
                {'speed_turn': 0.3}
            ]
        )
    ])
