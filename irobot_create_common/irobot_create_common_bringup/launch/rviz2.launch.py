# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch RViz.

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='robot namespace')
]


def generate_launch_description():
    create_bringup = get_package_share_directory(
        'irobot_create_common_bringup')

    namespace = LaunchConfiguration('namespace')

    # Rviz
    rviz_config = PathJoinSubstitution(
        [create_bringup, 'rviz', 'irobot_create_view.rviz'])
    rviz_logo = PathJoinSubstitution(
        [create_bringup, 'rviz', 'irobot_logo.jpg'])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '--display-config', rviz_config,
            '--splash-screen', rviz_logo,
        ],
        remappings=[
            ('/robot_description', (namespace, '/robot_description')),
            ('/standard_dock_description', (namespace, '/standard_dock_description'))
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    # Add nodes to LaunchDescription
    ld.add_action(rviz)

    return ld
