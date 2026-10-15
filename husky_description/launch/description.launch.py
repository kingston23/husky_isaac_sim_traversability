from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    declared_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock from /clock (published by Isaac Sim)",
        ),
        DeclareLaunchArgument(
            "use_gui",
            default_value="false",
            description="Start joint_state_publisher_gui. Set false when Isaac Sim "
                        "publishes /joint_states.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2.",
        ),
    ]

    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, use_sim_time],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        declared_args
        + [
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz,
        ]
    )