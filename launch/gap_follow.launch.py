import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("gap_follow_ver2"),
        "config",
        "gap_follow.yaml",
    )

    gap_follow_ver2_node = Node(
        package="gap_follow_ver2",
        namespace="gap_follow_ver2",
        executable="gap_follow_ver2",
        name="gap_follow_ver2",
        parameters=[config_path],
    )

    return LaunchDescription([gap_follow_ver2_node])
