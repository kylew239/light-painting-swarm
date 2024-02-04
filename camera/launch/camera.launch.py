from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="camera",
            executable="camera",
            ros_arguments=[
                "--params-file",
                PathJoinSubstitution([
                    FindPackageShare("camera"),
                    "config",
                    "camera.yaml"])
            ],
        )
    ])