"""Launch GNOCCHI for the low-altitude UAV (ZED VIO + single-antenna GNSS).

    ros2 launch gnocchi lowalt_uav.launch.py
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
        DeclareLaunchArgument("config", default_value="lowalt_uav.yaml"),
        Node(
            package="gnocchi",
            executable="gnocchi_node",
            namespace="lowalt_uav",
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
