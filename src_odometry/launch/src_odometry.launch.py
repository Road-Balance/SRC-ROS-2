import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    # Racecar controller launch
    src_odometry_gazebo = Node(
        package='src_odometry',
        # executable='racecar_controller',
        executable='src_odometry_gazebo',
        output='screen',
        parameters=[
            {
                "verbose": False,
                "open_loop": False,
            }
        ],
    )

    return LaunchDescription([
        src_odometry_gazebo,
    ])