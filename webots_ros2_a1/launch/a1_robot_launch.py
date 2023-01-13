from http.server import executable
import os
import pathlib

from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_a1')
    world = LaunchConfiguration('world')
    robot_description = pathlib.Path(os.path.join(
        package_dir, 'resource', 'webots_ros2_description_a1.urdf')).read_text()
    ros2_control_params = os.path.join(
        package_dir, 'resource', 'ros2_control_a1.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    a1_description_package_dir = get_package_share_directory('a1_description')
    default_rviz_config_path = os.path.join(
        a1_description_package_dir, 'rviz/unitree_a1_sim_show.rviz')
    # a1_description_urdf_path = os.path.join(
    #     a1_description_package_dir, 'urdf/robot_cut_description.urdf')
    
    a1_description_xacro_path = os.path.join(
        a1_description_package_dir, 'xacro/robot_cut.xacro')
    
    # with open(a1_description_urdf_path, 'r') as f:
    #     a1_description = f.read()
    
    # print("a1_description:*****************************************************************************************")
    # print(a1_description)

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
    
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Whether to use sim time'
    )

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]), mode='fast'
    )

    # ros2_supervisor = Ros2SupervisorLauncher()

    a1_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': 'UnitreeA1'},
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': False},
            ros2_control_params
        ],
    )

    # TODO: Revert once the https://github.com/ros-controls/ros2_control/pull/444 PR gets into the release
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    a1_effort_controllers_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_effort_controllers'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['a1_joint_state_broadcaster'] + controller_manager_timeout,
    )

    # camera_tf_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     output='screen',
    #     arguments=['0.2633438', '0', '0.0301362', '0', '0.2618', '0', 'trunk', 'range-finder'],
    # )

    lidar_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0.166', '0', '0', '0', 'trunk', 'lidar16'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', a1_description_xacro_path]),
            'publish_frequency': 200.0,
        }],
    )

    # Launch RViz
    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(a1_description_package_dir,
                         'launch', 'a1_sim_rviz2.launch.py')
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'rviz_config_file': rviz_config_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    start_rviz = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[
                LogInfo(msg='a1_robot_driver start'),
                TimerAction(
                    period=20.0,
                    actions=[start_rviz_cmd]
                )
            ],
        )
    )

    # delay_joint_state_broadcaster_start = launch.actions.RegisterEventHandler(
    #     event_handler=launch.event_handlers.OnProcessExit(
    #         target_action=a1_robot_driver,
    #         on_exit=[joint_state_broadcaster_spawner]
    #     )
    # )

    # delay_rviz_start = launch.actions.RegisterEventHandler(
    #     event_handler=launch.event_handlers.OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[start_rviz_cmd]
    #     )
    # )

    

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'world',
        #     default_value='unitree_a1_cut.wbt',
        #     description='Choose one of the world files from `/webots_ros2_a1/world` directory'
        # ),
        declare_use_rviz_cmd,
        declare_rviz_config_file_cmd,
        declare_use_sim_time,
        # webots,
        # ros2_supervisor,
        a1_robot_driver,
        # start_rviz_cmd,
        # # a1_effort_controllers_spawner,
        robot_state_publisher,
        
        joint_state_broadcaster_spawner,
        a1_effort_controllers_spawner,
        start_rviz,
        # start_webots,
        # camera_tf_publisher,
        lidar_tf_publisher,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=webots,
        #         on_exit=[launch.actions.EmitEvent(
        #             event=launch.events.Shutdown())],
        #     )
        # )
    ])
