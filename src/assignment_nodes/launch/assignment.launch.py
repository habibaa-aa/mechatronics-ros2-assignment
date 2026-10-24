
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
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
            parameters=[{
                'max_linear_speed': 2.0,
                'max_angular_speed': 4.0
            }],
            output='screen'
        ),
        Node(
            package='assignment_nodes',
            executable='draw_square_node',
            name='draw_square_node',
            output='screen'
        ),
    ])
