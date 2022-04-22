import os

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    mw_ahrs_node = Node(
        package='mw_ahrsv1_ros2',
        executable='mw_ahrsv1_only_angle',
        name='mw_ahrsv1_ros2',
        output='log',
        parameters=[{
            'deviceID' : '/dev/MWAHRs',
            'frame_id' : 'base_link',
            'child_frame_id' : 'imu_link',
            'publish_tf' : True,
            'view_imu' : False,
            'verbose' : False,
            'publish_rate' : 50,
        }],
    )

    rplidar_ros2_pkg = os.path.join(get_package_share_directory('rplidar_ros2'))
    serial_port = "/dev/rplidar"
    rplidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rplidar_ros2_pkg, 'launch', 'rplidar_a3_launch.py')),
        launch_arguments={'serial_port': serial_port}.items()
    )

    # x y z yaw pitch roll frame_id child_frame_id period_in_ms
    static_transform_publisher = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments=["0.24", "0", "0.14", "3.1415", "0", "0", "base_link", "laser"]
    )

    cmd_to_src = Node(
        package='cmd_to_src',
        executable='cmd_to_src',
        name='cmd_to_src',
        output='screen'
    )

    src_odom = Node(
        package='src_odometry',
        executable='src_odometry',
        name='src_odometry',
        output='log',
        parameters=[{
            "verbose" : False,
            'publish_rate' : 50,
            'open_loop' : False,
            'has_imu_heading' : True,
            'is_gazebo' : False,
            'wheel_radius' : 0.0508,
            'base_frame_id' : "base_link",
            'odom_frame_id' : "odom",
            'enable_odom_tf' : True,
        }],
    )

    return LaunchDescription([
        mw_ahrs_node,
        rplidar_driver,
        static_transform_publisher,
        cmd_to_src,
        src_odom,
    ])