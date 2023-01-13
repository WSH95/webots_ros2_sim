from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch.substitutions import EnvironmentVariable
# from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_a1')
    world = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='unitree_a1_cut.wbt',
        description='Choose one of the world files from `/webots_ros2_a1/world` directory'
    )
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]), mode='fast'
    )

    # ros2_supervisor = Ros2SupervisorLauncher()

    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', [EnvironmentVariable('WEBOTS_HOME'), '/lib/controller:', EnvironmentVariable('LD_LIBRARY_PATH')]),
        # ExecuteProcess(
        #     cmd=['echo', EnvironmentVariable('LD_LIBRARY_PATH')],
        #     output='screen'),
        declare_world,
        webots,
        # ros2_supervisor,
    ])
