from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os, yaml


def generate_launch_description():
    crazyflies_yaml = os.path.join(
        get_package_share_directory('light_painting'),
        'config',
        'config.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    # Get the names of the robot from the config file. This should be done
    # without scripting (ros convention), by formatting the config file to 
    # use ros__parameters, but I chose to do it this way to reduce modifications 
    # to the config files and nodes provided by Bitcraze
    robots=[]
    for robot in crazyflies['robots']:
        robots.append(robot)

    server_params = crazyflies
    return LaunchDescription([
        DeclareLaunchArgument('led_control',
                              default_value="radius",
                              description="Determines the LED controller mode" +
                              "(radius | line)"),
        DeclareLaunchArgument('threshold',
                              default_value="0.05",
                              description="The threshold for the LED controller to use"),

        # All drones (shared)
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            name='crazyflie_server',
            output='screen',
            parameters=[server_params],
            on_exit=Shutdown(),
        ),

        Node(package='light_painting',
            executable='waypoint',
            name='waypoint',
            on_exit=Shutdown(),
            parameters=[{'drones': robots}],
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("camera"),
                "launch",
                "camera.launch.py"
            ]),
        ),

        # First drone
        Node(
            package='light_painting',
            executable='flight',
            name='flight',
            parameters=[{'drone': robots[0]}],
            on_exit=Shutdown(),
        ),

        Node(
            package='light_painting',
            executable='led',
            name='led',
            on_exit=Shutdown(),
            parameters=[{'drone': robots[0]},
                        {'control': LaunchConfiguration('led_control')},
                        {'threshold': LaunchConfiguration('threshold')}],
        ),


])
