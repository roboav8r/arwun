"""Bring up robot_state_publisher for the Arwun rover.

Kept separate from record.launch.py so the description can be launched on its
own (e.g. alongside RViz) without starting the camera or the recorder.

The URDF is resolved at launch time rather than at file-parse time so that a
missing URDF degrades to a warning instead of taking down the whole launch --
useful while the description package is still a scaffold.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_description(context):
    urdf_path = LaunchConfiguration('urdf').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    if not os.path.isfile(urdf_path):
        return [LogInfo(msg=(
            f'[arwun_description] URDF not found at {urdf_path} -- skipping '
            'robot_state_publisher. No /tf or /tf_static will be published, '
            'so bags recorded now will NOT carry the camera-to-base '
            'transform. Drop arwun_dynamics.urdf into '
            'arwun_description/urdf/ and rebuild.'))]

    if urdf_path.endswith('.xacro'):
        import xacro
        robot_desc = xacro.process_file(urdf_path).toxml()
    else:
        with open(urdf_path, 'r') as f:
            robot_desc = f.read()

    return [
        LogInfo(msg=f'[arwun_description] loading URDF {urdf_path}'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time.lower() in ('true', '1'),
            }],
        ),
        # No joint_state_publisher: the rig has no actuated joints yet, so the
        # URDF is all-fixed and robot_state_publisher emits the complete tree
        # on /tf_static by itself. Add one here once real joints appear.
    ]


def generate_launch_description():
    default_urdf = os.path.join(
        get_package_share_directory('arwun_description'),
        'urdf', 'arwun_dynamics.urdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf', default_value=default_urdf,
            description='Absolute path to the robot URDF or .xacro.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock instead of wall time.'),
        OpaqueFunction(function=_robot_description),
    ])
