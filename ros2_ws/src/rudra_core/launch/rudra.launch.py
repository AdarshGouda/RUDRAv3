from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rudra_core',
            executable='rudra_alive',
            name='rudra_alive',
            output='screen',
        ),
        Node(
            package='rudra_core',
            executable='odom_tf_broadcaster',
            name='odom_tf_broadcaster',
            output='screen',
        ),
    ])
