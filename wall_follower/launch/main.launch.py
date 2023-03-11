from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='wall_finder_node',
            output='screen'),
        Node(
            package='wall_follower',
            executable='odom_recorder_server_node',
            output='screen'),
        Node(
            package='wall_follower',
            executable='wall_follower_node_part3',
            output='screen')
    ])