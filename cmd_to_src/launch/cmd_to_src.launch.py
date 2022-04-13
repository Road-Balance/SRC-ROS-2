import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    cmd_to_src = Node(
        package='cmd_to_src',
        executable='cmd_to_src',
        name='cmd_to_src',
        output='screen',
        parameters=[{
            "p_gain" : 122.0,
            'i_gain' : 3.0,
            'd_gain' : 1.0,
            'use_twiddle' : False,
        }],
    )

    return LaunchDescription([
        cmd_to_src
    ])