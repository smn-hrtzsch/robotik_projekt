from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Haupt-Node (line_follower)
    line_follower_node = Node(
        package='line_follower',
        executable='line_follower',
        name='line_follower',
        output='screen'
    )

    # Stop-Node
    stop_node = Node(
        package='line_follower',
        executable='stop',
        name='stop',
        output='screen'
    )

    return LaunchDescription([
        line_follower_node,
        stop_node
    ])
