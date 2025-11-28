from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Direct path to assignment_1.launch.py in your workspace source
    assignment_launch = os.path.join(
        os.path.expanduser('~/ws_8_assignments/src/ir_2526/ir_launch/launch'),
        'assignment_1.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(assignment_launch)
        ),

        Node(
            package='group_8_assignment_1',
            executable='tag_goal_node',
            output='screen'
        ),
        Node(
            package='group_8_assignment_1',
            executable='table_detection_node',
            output='screen'
        ),
    ])

