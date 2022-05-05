# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='robot namespace'),
    DeclareLaunchArgument('use_namespace', default_value='False',
                          description='robot namespace'),
]


def generate_launch_description():
    create_bringup = get_package_share_directory(
        'irobot_create_common_bringup')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    # Rviz
    rviz_config = PathJoinSubstitution(
        [create_bringup, 'rviz', 'irobot_create_view.rviz'])
    rviz_logo = PathJoinSubstitution(
        [create_bringup, 'rviz', 'irobot_logo.jpg'])

    rviz = Node(
        condition=UnlessCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '--display-config', rviz_config,
            '--splash-screen', rviz_logo,
        ]
    )

    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
                '--display-config', rviz_config,
                '--splash-screen', rviz_logo
        ],
        output='screen',
        remappings=[
            ('/robot_description', (namespace, '/robot_description'))
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    # Add nodes to LaunchDescription
    ld.add_action(rviz)
    ld.add_action(start_namespaced_rviz_cmd)

    return ld
