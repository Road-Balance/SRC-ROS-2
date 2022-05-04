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
            "accel_scale" : 0.1,
            "deaccel_scale" : 0.1,
            "scale" : 24, # linear 하지 못하다. 0.1에서는 좀 못미치고, 0.4에서는 넘어버린다.
            "p_gain" : 122.0,
            'i_gain' : 3.0,
            'd_gain' : 1.0,
            'use_twiddle' : False,
        }],
    )

    return LaunchDescription([
        cmd_to_src
    ])