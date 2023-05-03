import os
import launch_ros
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # load one controller just to make sure it can connect to controller_manager
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # prbt_joint_1_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["prbt_joint_1_controller", "--controller-manager", "/controller_manager"],
    # )

    # prbt_joint_2_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["prbt_joint_2_controller", "--controller-manager", "/controller_manager"],
    # )    
    
    # prbt_joint_3_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["prbt_joint_3_controller", "--controller-manager", "/controller_manager"],
    # )

    # prbt_joint_4_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["prbt_joint_4_controller", "--controller-manager", "/controller_manager"],
    # )

    # prbt_joint_5_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["prbt_joint_5_controller", "--controller-manager", "/controller_manager"],
    # )    
    
    # prbt_joint_6_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["prbt_joint_6_controller", "--controller-manager", "/controller_manager"],
    # )

    prbt_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["prbt_joint_controller", "--controller-manager", "/controller_manager"],
    )

    # forward_position_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    # )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )


    return LaunchDescription(
        [
            # joint_state_broadcaster_spawner,
            # prbt_joint_1_controller_spawner,
            # prbt_joint_2_controller_spawner,
            # prbt_joint_3_controller_spawner,
            # prbt_joint_4_controller_spawner,
            # prbt_joint_5_controller_spawner,
            # prbt_joint_6_controller_spawner,
            prbt_joint_controller_spawner,
            # forward_position_controller,
            arm_controller_spawner,
        ],
    )