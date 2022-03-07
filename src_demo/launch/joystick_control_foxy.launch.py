import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    joy_to_cmd_vel = Node(
        package='joy_to_cmd',
        executable='joy_to_cmd_vel',
        name='joy_to_cmd_vel',
        output='screen'
    )

    cmd_to_src = Node(
        package='cmd_to_src',
        executable='cmd_to_src',
        name='cmd_to_src',
        output='screen'
    )

    return LaunchDescription([
        joy,
        joy_to_cmd_vel,
        cmd_to_src,
    ])