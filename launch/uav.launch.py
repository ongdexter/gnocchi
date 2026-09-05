"""Launch GNOCCHI for the UAV (GPS-only: no odometry, no graph).

    ros2 launch gnocchi uav.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("config", default_value="uav.yaml"),
        Node(
            package="gnocchi",
            executable="gnocchi_node",
            namespace="uav",
            name="gnocchi",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("gnocchi"),
                    "config",
                    LaunchConfiguration("config"),
                ]),
                {"use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool)},
            ],
        ),
    ])
