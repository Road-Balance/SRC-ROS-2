import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

import xacro

def generate_launch_description():

    # gazebo
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    pkg_gazebo_ros, 'launch', 'gazebo.launch.py'))
            )

    # Robot State Publisher
    pkg_path = os.path.join(get_package_share_directory('src_gazebo'))
    urdf_file = os.path.join(pkg_path, 'urdf', 'src_ackermann.urdf')

    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Spawn Robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'racecar'],
                        output='screen')

    # ROS 2 controller
    load_forward_position_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller"],
        output="screen",
    )

    load_velocity_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["velocity_controller"],
        output="screen",
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Racecar controller launch
    racecar_control = Node(
        package='src_gazebo_controller',
        executable='racecar_controller',
        output='screen',
        parameters=[
            {
                "verbose": False,
            }
        ],
    )

    # rqt robot steering
    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_forward_position_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_forward_position_controller,
                on_exit=[load_velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_velocity_controller,
                on_exit=[racecar_control],
            )
        ),
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        rqt_robot_steering,
    ])