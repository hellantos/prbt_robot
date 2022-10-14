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

    slave_eds_path = os.path.join(
                    get_package_share_directory("prbt_robot_support"), "config", "prbt", "prbt_0_1.dcf"
                )

    for i in range(1,7):

        slave_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("canopen_fake_slaves"), "launch"
                    ),
                    "/cia402_slave.launch.py",
                ]
            ),
            launch_arguments={
                "node_id": "{}".format(i+2), 
                "node_name": "prbt_slave_{}".format(i),
                "slave_config": slave_eds_path,
                }.items(),
        )
        ld.add_action(slave_node)
    master_bin_path = os.path.join(
                get_package_share_directory("prbt_robot_support"),
                "config",
                "prbt",
                "master.bin",
            )     
    if not os.path.exists(master_bin_path):
        master_bin_path = ""        
    device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": os.path.join(
                get_package_share_directory("prbt_robot_support"),
                "config",
                "prbt",
                "master.dcf",
            ),
            "master_bin": master_bin_path,
            "bus_config": os.path.join(
                get_package_share_directory("prbt_robot_support"),
                "config",
                "prbt",
                "bus.yml",
            ),
            "can_interface_name_name": "vcan0",
        }.items(),
    )

    prbt_xacro_file = os.path.join(get_package_share_directory('prbt_robot_support'), 'urdf',
                                     'prbt.xacro')
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), 
            ' ', 
            prbt_xacro_file
        ])

    #rviz_file = os.path.join(get_package_share_directory('prbt_robot_support'), 'rviz',
    #                         'visualize_franka.rviz')

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': launch_ros.descriptions.ParameterValue(value=robot_description, value_type=str)}],
        )
    rviz2 = Node(package='rviz2',
             executable='rviz2',
             name='rviz2')


    state_publisher = Node(
        package="joint_state_publisher",
        name="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{
            "source_list": [
                "/prbt_joint_1/joint_states",
                "/prbt_joint_2/joint_states",
                "/prbt_joint_3/joint_states",
                "/prbt_joint_4/joint_states",
                "/prbt_joint_5/joint_states",
                "/prbt_joint_6/joint_states",
                "/prbt_joint_7/joint_states",
                ],
            "rate": 10
        }]
    )

    ld.add_action(device_container)
    ld.add_action(robot_state_publisher)
    ld.add_action(state_publisher)
    ld.add_action(rviz2)
    return ld
