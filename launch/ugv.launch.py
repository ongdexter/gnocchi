"""Launch GNOCCHI for the UGV.

    ros2 launch gnocchi ugv.launch.py
    ros2 launch gnocchi ugv.launch.py use_sim_time:=true
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
        DeclareLaunchArgument("config", default_value="ugv.yaml"),
        Node(
            package="gnocchi",
            executable="gnocchi_node",
            namespace="ugv",
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
