from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion_control_pkg',
            executable='arm_control_node',
            name='arm_control_node'
        ),
        Node(
            package='camera_pkg',
            executable='pouring_accuracy_node',
            name='pouring_accuracy_node'
        ),
        Node(
            package='camera_pkg',
            executable='bottle_recog_node',
            name='bottle_recog_node'
        ),
        Node(
            package='camera_pkg',
            executable='bottle_recog_node',
            name='bottle_recog_node'
        ),
        Node(
            package='motion_control_pkg',
            executable='pouring_node',
            name='pouring_node'
        ),
        Node(
            package='mic_pkg',
            executable='mic_node',
            name='mic_node'
        )
    ])
