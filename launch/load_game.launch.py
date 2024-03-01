
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gators'),
        'config',
        'params.yaml'
    )
    rviz_config = os.path.join(
        get_package_share_directory('gators'),
        'rviz',
        'config.rviz'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='set the log level for the node',
        ),
        Node(
            package='gators',
            namespace='gators',
            executable='game_player',
            name='game_player',
            output='screen',
            parameters=[config],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
        Node(
            package='rviz2',
            namespace='gators',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
