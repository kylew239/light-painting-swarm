from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from ament_index_python.packages import get_package_share_directory
import os, yaml


def generate_launch_description():
    crazyflies_yaml = os.path.join(
        get_package_share_directory('light_painting'),
        'config',
        'config.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    server_params = crazyflies
    return LaunchDescription([
        # Args
        # DeclareLaunchArgument("crazyflies_yaml",
        #                       default_value=PathJoinSubstitution([FindPackageShare('light_painting'),
        #                                       'config',
        #                                       'config.yaml'])),

        DeclareLaunchArgument('led_control',
                              default_value="radius",
                              description="Determines the LED controller mode" +
                              "(radius | line)"),
        DeclareLaunchArgument('threshold',
                              default_value="0.05",
                              description="The threshold for the LED controller to use"),

        # Nodes
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            name='crazyflie_server',
            output='screen',
            parameters=[server_params],
            on_exit=Shutdown(),
        ),

        Node(
            package='light_painting',
            executable='flight',
            name='flight',
            on_exit=Shutdown(),
        ),

        Node(
            package='light_painting',
            executable='led',
            name='led',
            on_exit=Shutdown(),
            arguments=['control', LaunchConfiguration('led_control'),
                       'threshold', LaunchConfiguration('threshold')],
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("camera"),
                "launch",
                "camera.launch.py"
            ]),
        ),

])
