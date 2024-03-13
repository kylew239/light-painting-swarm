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
                              default_value="0.03",
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
            name='flight_' + robots[0],
            parameters=[{'drone': robots[0]},
                        {'y_offset': -0.15}],
            on_exit=Shutdown(),
        ),

        Node(
            package='light_painting',
            executable='led',
            name='led_' + robots[0],
            on_exit=Shutdown(),
            parameters=[{'drone': robots[0]},
                        {'control': LaunchConfiguration('led_control')},
                        {'threshold': LaunchConfiguration('threshold')},
                        {'color': 'red'}],
        ),

        # Second drone
        Node(
            package='light_painting',
            executable='flight',
            name='flight_' + robots[1],
            parameters=[{'drone': robots[1]},
                        {'y_offset': -0.15}],
            on_exit=Shutdown(),
        ),

        Node(
            package='light_painting',
            executable='led',
            name='led_' + robots[1],
            on_exit=Shutdown(),
            parameters=[{'drone': robots[1]},
                        {'control': LaunchConfiguration('led_control')},
                        {'threshold': LaunchConfiguration('threshold')},
                        {'color': 'green'}],
        ),

        # Third drone
        Node(
            package='light_painting',
            executable='flight',
            name='flight_' + robots[2],
            parameters=[{'drone': robots[2]},
                        {'y_offset': -0.15}],
            on_exit=Shutdown(),
        ),

        Node(
            package='light_painting',
            executable='led',
            name='led_' + robots[2],
            on_exit=Shutdown(),
            parameters=[{'drone': robots[2]},
                        {'control': LaunchConfiguration('led_control')},
                        {'threshold': LaunchConfiguration('threshold')},
                        {'color': 'blue'}],
        ),
])
