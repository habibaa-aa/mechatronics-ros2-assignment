
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='assignment_nodes',
            executable='reset_node',
            name='reset_node',
            output='screen'
        ),
        Node(
            package='assignment_nodes',
            executable='go_to_pose_node',
            name='go_to_pose_node',
            output='screen'
        ),
        Node(
            package='assignment_nodes',
            executable='draw_square_node',
            name='draw_square_node',
            output='screen'
        ),
    ])
