from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_follower_bo',
            executable='line_follower',
            name='line_follower',
            output='screen'
        ),
        Node(
            package='line_follower_bo',
            executable='handle_obstacle',
            name='handle_obstacle',
            output='screen'
        ),
        Node(
            package='line_follower_bo',
            executable='state_manager',
            name='state_manager',
            output='screen'
        ),
    ])
