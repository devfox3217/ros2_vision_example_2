from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lane_detection',
            executable='video_publisher',
            name='video_publisher',
            output='screen'
        ),
        Node(
            package='lane_detection',
            executable='lane_processor',
            name='lane_processor',
            output='screen'
        ),
        Node(
            package='lane_detection',
            executable='visualizer_node',
            name='visualizer_node',
            output='screen'
        ),
        Node(
            package='lane_detection',
            executable='gui_node',
            name='gui_node',
            output='screen'
        ),
    ])