import os
import launch_ros
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    prbt_xacro_file = os.path.join(get_package_share_directory('prbt_description'), 'urdf',
                                     'prbt.xacro')

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), 
            ' ', 
            prbt_xacro_file
        ])

    rviz_file = os.path.join(get_package_share_directory('prbt_description'), 'launch',
                            'basic.rviz')

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': launch_ros.descriptions.ParameterValue(value=robot_description, value_type=str)}],
        )
    
    rviz2 = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             output='screen',
             arguments=['-d', rviz_file],
             parameters=[
                {'robot_description': launch_ros.descriptions.ParameterValue(value=robot_description, value_type=str)},
             ])

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[
            {'robot_description': launch_ros.descriptions.ParameterValue(value=robot_description, value_type=str)},
        ],
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2)
    return ld
