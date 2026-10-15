#!/usr/bin/env python3
"""
Launches dual-EKF localisation with a simulated GPS.

    ros2 launch <package> dual_ekf_navsat.launch.py

Prerequisites in Isaac Sim:
    - husky_odom_tf_graph.py with PUBLISH_ODOM_TF = False
    - /clock is being published
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# change to your own package name
PKG = "husky_localization"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    params = os.path.join(
        get_package_share_directory(PKG), "config", "dual_ekf_navsat.yaml"
    )

    return LaunchDescription([

        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # ---------- odometry drift simulator ----------
        # Turns Isaac's ground-truth /odom into a realistically drifting
        # /odom/noisy. Without this the EKF has nothing to correct.
        Node(
            package=PKG,
            executable="odom_drift_node",
            name="odom_drift",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": "/odom",          # Isaac ground truth
                "output_topic": "/odom/noisy",   # what the EKFs consume
                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_tf": False,             # the local EKF owns this TF
                "linear_scale": 1.02,
                "angular_scale": 0.93,
                "noise_trans_trans": 0.02,
                "noise_trans_rot": 0.05,
                "noise_rot_rot": 0.06,
                "noise_rot_trans": 0.02,
                "lateral_slip": 0.04,
            }],
        ),

        # ---------- simulated GPS ----------
        Node(
            package=PKG,
            executable="gps_sim_node",
            name="gps_sim",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                # IMPORTANT: this must be GROUND TRUTH from Isaac, not wheel
                # odometry. Otherwise the GPS "sees" the same drift it is
                # supposed to correct, and the whole exercise is meaningless.
                "ground_truth_topic": "/odom",   # raw Isaac ground truth, NOT /odom/noisy
                "fix_topic": "/gps/fix",
                "frame_id": "gps_link",
                "datum_latitude": 45.8150,
                "datum_longitude": 15.9819,
                "datum_altitude": 120.0,
                "publish_rate": 5.0,
                "noise_stddev_xy": 0.8,
                "drift_stddev": 0.05,
                "antenna_offset_z": 0.3,
            }],
        ),

        # ---------- gps_link frame ----------
        # The URDF has no gps_link (it was commented out), so we add it here.
        # Arguments: x y z yaw pitch roll parent child
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="gps_link_tf",
            arguments=["0", "0", "0.3", "0", "0", "0", "base_link", "gps_link"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # ---------- local EKF: odom -> base_link ----------
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[params, {"use_sim_time": use_sim_time}],
            remappings=[("odometry/filtered", "odometry/filtered/local")],
        ),

        # ---------- global EKF: map -> odom ----------
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=[params, {"use_sim_time": use_sim_time}],
            remappings=[("odometry/filtered", "odometry/filtered/global")],
        ),

        # ---------- navsat_transform ----------
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[params, {"use_sim_time": use_sim_time}],
            remappings=[
                ("imu", "imu"),
                ("gps/fix", "gps/fix"),
                ("odometry/filtered", "odometry/filtered/global"),
                ("odometry/gps", "odometry/gps"),
                ("gps/filtered", "gps/filtered"),
            ],
        ),
    ])
