"""Bring up the Arwun field data-collection rig.

Starts the D435i, the joystick, the robot description, and the joystick-driven
rosbag2 recorder. Nothing records until the toggle button is pressed -- the
recorder comes up idle so you can verify topics before burning disk.

    ros2 launch arwun_bringup record.launch.py

Useful overrides:

    output_dir:=/media/ssd/bags     write bags somewhere other than ~/arwun_bags
    joy_dev:=/dev/input/js1         if the controller enumerates elsewhere
    camera:=false                   bring up everything but the D435i
    urdf:=/abs/path/to.urdf         non-default robot description
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    description_share = get_package_share_directory('arwun_description')

    params_file = LaunchConfiguration('params_file').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    joy_dev = LaunchConfiguration('joy_dev').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)

    actions = []

    # --- camera -------------------------------------------------------
    actions.append(Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(LaunchConfiguration('camera')),
    ))

    # --- joystick -----------------------------------------------------
    # joy_dev is overridden after the params file so the launch argument wins.
    actions.append(Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux',
        output='screen',
        parameters=[params_file, {'dev': joy_dev}],
        condition=IfCondition(LaunchConfiguration('joy')),
    ))

    # --- robot description --------------------------------------------
    # Supplies /tf and /tf_static so recorded bags carry camera-to-base.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'description.launch.py')),
        launch_arguments={'urdf': LaunchConfiguration('urdf')}.items(),
        condition=IfCondition(LaunchConfiguration('description')),
    ))

    # --- recorder -----------------------------------------------------
    actions.append(Node(
        package='arwun_teleop',
        executable='record_controller',
        name='record_controller',
        output='screen',
        parameters=[params_file, {'output_dir': output_dir}],
        arguments=['--ros-args', '--log-level', log_level],
    ))

    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory('arwun_bringup')
    description_share = get_package_share_directory('arwun_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                bringup_share, 'config', 'record_params.yaml'),
            description='YAML with camera, joystick and recorder parameters.'),
        DeclareLaunchArgument(
            'output_dir', default_value='~/arwun_bags',
            description='Directory that bags are written into.'),
        DeclareLaunchArgument(
            'joy_dev', default_value='/dev/input/js0',
            description='Joystick device node.'),
        DeclareLaunchArgument(
            'urdf',
            default_value=os.path.join(
                description_share, 'urdf', 'arwun_dynamics.urdf'),
            description='Robot description used for /tf and /tf_static.'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Start the RealSense driver.'),
        DeclareLaunchArgument(
            'joy', default_value='true',
            description='Start the joystick driver.'),
        DeclareLaunchArgument(
            'description', default_value='true',
            description='Start robot_state_publisher.'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='ROS log level for the camera and recorder nodes.'),
        OpaqueFunction(function=_setup),
    ])
