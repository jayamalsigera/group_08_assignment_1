from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown
from launch.actions import ExecuteProcess, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Declare the use_sim_time launch argument
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
        launch_arguments={'use_sim_time': use_sim_time, 'autostart': 'True'}.items()
    )
    
    # Get the directory of the apriltag_ros package
    apriltag_pkg = get_package_share_directory("apriltag_ros")

    # Include the apriltag detector launch file
    apriltag_detector = IncludeLaunchDescription(
        YAMLLaunchDescriptionSource(
            os.path.join(apriltag_pkg, 'launch', 'camera_36h11.launch.yml')
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "tag_size": "0.050"
        }.items()
    )

    # Define the camera relay node
    relay_node = Node(
        package="group_8_assignment_1",
        executable="camera_relay_node",
        name="camera_relay_node",
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )

    # Define the initial pose node
    initial_pose_node = Node(
        package='group_8_assignment_1',
        executable='initial_pose_node',
        name='initial_pose_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Define the apriltag center node
    apriltag_center_node = Node(
        package='group_8_assignment_1',
        executable='apriltag_center_node',
        name='apriltag_center_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Define the navigation goal node
    navigation_goal_node = Node(
        package='group_8_assignment_1',
        executable='navigation_goal_node',
        name='navigation_goal_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'publish_rate': 10.0}
        ]
    )

    kill_gazebo_on_exit = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd="pkill -9 gz", shell=True
                ),
                ExecuteProcess(
                    cmd="pkill -9 gzserver", shell=True
                ),
                ExecuteProcess(
                    cmd="pkill -9 gzclient", shell=True
                ),
                ExecuteProcess(
                    cmd="pkill -9 ign", shell=True
                ),
                ExecuteProcess(
                    cmd="pkill -9 ruby", shell=True 
                )
            ]
        )
    )  

    return LaunchDescription([
        declare_use_sim_time,
        assignment_1_launch,
        apriltag_detector,
        relay_node,
        apriltag_center_node,
        initial_pose_node,
        navigation_goal_node,
        # kill_gazebo_on_exit
    ])
