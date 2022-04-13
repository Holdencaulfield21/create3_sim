from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch.actions
import launch_ros.actions


def generate_launch_description():

    pkg_create3_common_bringup = get_package_share_directory('irobot_create_common_bringup')
    robot_description_launch = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'robot_description.launch.py'])

    return LaunchDescription([
        launch_ros.actions.Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            arguments=[
                '-name', launch.substitutions.LaunchConfiguration('robot_name'),
                '-topic', launch.substitutions.LaunchConfiguration('robot_description'),
                '-Y', '0.0',
                '-x', launch.substitutions.LaunchConfiguration('x'),
                '-y', launch.substitutions.LaunchConfiguration('y'),
                '-z', launch.substitutions.LaunchConfiguration('z')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch]),
            launch_arguments={'gazebo': 'ignition', 'namespace': launch.substitutions.LaunchConfiguration('robot_name')}.items())
            
    ])