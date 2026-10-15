from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('husky_traversability'), 'config', 'ground_segmentation.yaml'
            ]),
            description='Ground segmentation parameters'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='husky_traversability',
            executable='ground_segmentation_node',
            name='ground_segmentation',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
