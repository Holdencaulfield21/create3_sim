# Copyright 2021 Clearpath Robotics, Inc.
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution


class OffsetParser(Substitution):
    def __init__(
            self,
            number: SomeSubstitutionsType,
            offset: float,
    ) -> None:
        self.__number = number
        self.__offset = offset

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        number = float(self.__number.perform(context))
        return f'{number + self.__offset}'


ARGUMENTS = [
    DeclareLaunchArgument('bridge', default_value='true',
                          choices=['true', 'false'],
                          description='Use ros_ign_bridge'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='Ignition World'),
    DeclareLaunchArgument('robot_name', default_value='create3',
                          description='Robot name'),
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('spawn_dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def gen_robot_list(number_of_robots):

    robots = []

    for i in range(number_of_robots):
        robot_name = "create3_"+str(i)
        x_pos = float(i)
        robots.append({'name': robot_name, 'robot_description': robot_name+'/robot_description', 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 0.01})


    return robots 


def generate_launch_description():

    # Directories
    pkg_irobot_create_common_bringup = get_package_share_directory(
        'irobot_create_common_bringup')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')
    pkg_irobot_create_ignition_plugins = get_package_share_directory(
        'irobot_create_ignition_plugins')
    pkg_irobot_create_description = get_package_share_directory(
        'irobot_create_description')
    pkg_ros_ign_gazebo = get_package_share_directory(
        'ros_ign_gazebo')

    # Set Ignition resource path
    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
                                               value=[os.path.join(
                                                      pkg_irobot_create_ignition_bringup,
                                                      'worlds'), ':' +
                                                      str(Path(
                                                          pkg_irobot_create_description).
                                                          parent.resolve())])

    ign_gui_plugin_path = SetEnvironmentVariable(name='IGN_GUI_PLUGIN_PATH',
                                                 value=[os.path.join(
                                                        pkg_irobot_create_ignition_plugins,
                                                        'lib')])

    # Paths
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])
    ros_ign_bridge_launch = PathJoinSubstitution(
        [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ros_ignition_bridge.launch.py'])
    create3_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'create3_nodes.launch.py'])
    create3_ignition_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ignition_nodes.launch.py'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'robot_description.launch.py'])
    dock_description_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'dock_description.launch.py'])
    rviz2_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'rviz2.launch.py'])

    # Launch configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # Ignition gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('ign_args', [LaunchConfiguration('world'),
                          '.sdf',
                          ' -v 4',
                          ' --gui-config ',
                          PathJoinSubstitution([pkg_irobot_create_ignition_bringup,
                                                'gui', 'create3', 'gui.config'])])
        ]
    )

    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz2_launch]),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # x_dock = OffsetParser(x, 0.157)
    # yaw_dock = OffsetParser(yaw, 3.1416)
    # dock_description = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([dock_description_launch]),
    #     condition=IfCondition(LaunchConfiguration('spawn_dock')),
    #     # The robot starts docked
    #     launch_arguments={'x': x_dock, 'y': y, 'z': z, 'yaw': yaw_dock,
    #                       'gazebo': 'ignition'}.items(),
    # )

    # robot_description = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([robot_description_launch]),
    #     launch_arguments={'gazebo': 'ignition'}.items()
    # )

    robots = gen_robot_list(1)
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_irobot_create_ignition_bringup, 'launch',
                                                           'create3_multi_spawn.launch.py')),
                launch_arguments={
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'robot_name': robot['name'],
                                  'robot_description': robot['robot_description']
                                  }.items()))

    # Dock
    # spawn_dock = Node(package='ros_ign_gazebo', executable='create',
    #                   arguments=['-name', 'standard_dock',
    #                              '-x', x_dock,
    #                              '-y', y,
    #                              '-z', z,
    #                              '-Y', '3.141592',
    #                              '-topic', 'standard_dock_description'],
    #                   output='screen')

    # ROS Ign bridge
    ros_ignition_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ros_ign_bridge_launch]),
                launch_arguments=[('world', LaunchConfiguration('world')),
                                ('robot_name', LaunchConfiguration('robot_name')),
                                ('namespace', robot['name'])]
            ))

    # ros_ign_bridge = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([ros_ign_bridge_launch]),
    #     launch_arguments=[('world', LaunchConfiguration('world')),
    #                       ('robot_name', LaunchConfiguration('robot_name'))]
    # )

    # Create3 nodes
    create3_nodes_cmds = []
    for robot in robots:
        create3_nodes_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([create3_nodes_launch]),
                launch_arguments=[('namespace', robot['name'])]
            ))

    create3_ignition_nodes_cmds = []
    for robot in robots:
        create3_nodes_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([create3_ignition_nodes_launch]),
                launch_arguments=[('robot_name', LaunchConfiguration('robot_name')),
                                  ('namespace', robot['name'])]
            ))

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gui_plugin_path)
    ld.add_action(ignition_gazebo)
    for ros_ignition_cmd in ros_ignition_cmds:
        ld.add_action(ros_ignition_cmd)
    ld.add_action(rviz2)
    # ld.add_action(dock_description)
    for spawn_robots_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robots_cmd)
    # ld.add_action(spawn_dock)

    for create3_nodes_cmd in create3_nodes_cmds:
        ld.add_action(create3_nodes_cmd)
    for create3_ignition_nodes_cmd in create3_ignition_nodes_cmds:
        ld.add_action(create3_ignition_nodes_cmd)
    return ld
