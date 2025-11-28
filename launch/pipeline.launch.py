from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )

    # Get the directory of the ir_launch package
    ir_launch_dir = get_package_share_directory('ir_launch')

    # Include the assignment_1.launch.py from the ir_launch package
    assignment_1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ir_launch_dir, 'launch', 'assignment_1.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    apriltag_pkg = get_package_share_directory("apriltag_ros")

    apriltag_detector = IncludeLaunchDescription(
        YAMLLaunchDescriptionSource(
            os.path.join(apriltag_pkg, 'launch', 'camera_36h11.launch.yml')
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "tag_size": "0.050"
        }.items()
    )

    relay_node = Node(
        package="group_8_assignment_1",
        executable="camera_relay_node",
        name="camera_relay_node",
        output="screen"
    )


    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            {"use_sim_time": True},
            {"scan_topic": "/scan"},
            {"min_particles": 200},
            {"max_particles": 2000},
            {"odom_frame_id": "odom"},
            {"base_frame_id": "base_link"},
            {"global_frame_id": "map"},
            {"tf_broadcast": True}
        ],
        output='screen',
    )

    initial_pose_node = Node(
        package='group_8_assignment_1',
        executable='initial_pose_node',
        name='initial_pose_node',
        parameters=[{"use_sim_time": True}],
        output='screen'
    )


    apriltag_center_node = Node(
        package='group_8_assignment_1',
        executable='apriltag_center_node',
        name='apriltag_center_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        assignment_1_launch,
        apriltag_detector,
        relay_node,
        amcl_node,
        apriltag_center_node,
        initial_pose_node
    ])