import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to this package.
    pkg_share = FindPackageShare(
        package='a1_description').find('a1_description')

    # Set the path to the RViz configuration settings
    default_rviz_config_path = os.path.join(
        pkg_share, 'rviz/unitree_a1_sim_show.rviz')

    ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############
    # Launch configuration variables specific to simulation

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Declare the launch arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Whether to use sim time'
    )

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time,}
        ],
    )

    return LaunchDescription([
        declare_rviz_config_file_cmd,
        declare_use_rviz_cmd,
        declare_use_sim_time,
        start_rviz_cmd,
    ])
