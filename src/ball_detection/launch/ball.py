from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ball_detection',  # Replace with your package name
            executable='ball_detector',
            output='screen',
        ),
        Node(
            package='ball_detection',
            executable='ball_finder',
            output='screen',

        ),
        

    ])
