"""

"""

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # We need to get the path to the .rviz file...
    this_prefix = get_package_share_directory('jmu_turtlebot3_bringup')
    rviz_path = os.path.join(this_prefix, 'rviz', 'tb3_navigation2.rviz')

    return LaunchDescription([
        Node(
            package="jmu_turtlebot3_bringup",
            node_executable="tb_fixer",
            node_name="tb_fixer",
            output="screen",
        ),
        Node(
            package="rviz2",
            node_executable="rviz2",
            node_name="rviz2",
            output="screen",
            arguments=['-d', rviz_path]
        )

    ])
