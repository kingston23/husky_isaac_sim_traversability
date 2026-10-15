from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, GroupAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from pathlib import Path
import os
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
]


def generate_launch_description():
    # Directories
    gps_wpf_dir = get_package_share_directory(
        "husky_navigation")

    localization_dir = get_package_share_directory(
        "husky_localization")
    localization_launch_dir = os.path.join(localization_dir, 'launch')    

    params_dir = os.path.join(gps_wpf_dir, "config")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')    
    nav2_params = os.path.join(params_dir, "nav2.yaml")
    twist_params =  os.path.join(params_dir, "twist_mux.yaml")
    # contollers_params = os.path.join(params_dir, "husky_controllers.yaml")

    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )


    # Launch args
    world_path = LaunchConfiguration('world_path')
    prefix = LaunchConfiguration('prefix')

    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_navigation"), "config", "husky_controllers.yaml"]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
            ),
            " ",
            "name:=husky",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            "gazebo_controllers:=",
            config_husky_velocity_controller,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    spawn_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_husky_velocity_controller, robot_description,
                    {'use_sim_time': True}],
        output='screen',
    )

    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        # remappings = [
        #     ( 'cmd_vel_nav','/husky_velocity_controller/cmd_vel'),
        # ],
        output='screen',
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}, robot_description],
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    
    # Node for the interactive marker twist server
    interactive_marker_node = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_marker_server',
        remappings=[
        ('cmd_vel', 'twist_marker_server/cmd_vel')
        ],
        output='screen'
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[{'use_sim_time': False}, twist_params],
        remappings=[
         ('cmd_vel_out', 'husky_velocity_controller/cmd_vel')
        ],
        output='screen',
    )

    # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_husky_velocity_controller],
        )
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_launch_dir, 'dual_ekf_navsat.launch.py'))
    )


    #nav2
    launch_nav2 = PathJoinSubstitution(
      [gps_wpf_dir, 'launch', 'navigation_launch.py'])

    # Navigation2 Launch Group
    nav2 = GroupAction([
        # Remap topics for global and local costmaps
        SetRemap('/global_costmap/scan', '/lidar/scan'),
        SetRemap('/local_costmap/scan', '/lidar/scan'),
        SetRemap('/local_costmap/lidar/point_cloud', '/lidar/point_cloud'),

        # SetRemap('/local_costmap/ground_segmentation/ground', '/ground_segmentation/ground'),
        # SetRemap('/local_costmap/ground_segmentation/obstacles', '/ground_segmentation/obstacles'),


        SetRemap('/cmd_vel_smoothed', '/cmd_vel'),

        # Include the navigation launch description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': configured_params,
                'autostart': 'true',
                #'use_composition': 'false',
            }.items(),
        ),
    ])

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "top_plate_front_link", "OS0"],
    )

    ld = LaunchDescription(ARGUMENTS)
    # robot localization launch
    ld.add_action(robot_localization_cmd)

    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(diffdrive_controller_spawn_callback)
    # ld.add_action(spawn_controller_manager)

    ld.add_action(static_tf_node)

    # navigation2 launch
    ld.add_action(nav2)
    
    #twist_mux node
    # ld.add_action(interactive_marker_node)
    # ld.add_action(twist_mux_node)


    return ld
