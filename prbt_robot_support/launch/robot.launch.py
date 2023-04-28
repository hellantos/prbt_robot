import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros

def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            description="Package where urdf file is stored.",
            default_value="prbt_description"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="vcan0",
            description="Interface name for can",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="true",
            description="Use ros2_control",
        )
    )

    controller_config = PathJoinSubstitution([FindPackageShare("prbt_robot_support"), "config", "prbt_ros2_control.yaml"])
    bus_config = PathJoinSubstitution([FindPackageShare("prbt_robot_support"), "config", "prbt", "bus.yml"])
    master_config = PathJoinSubstitution([FindPackageShare("prbt_robot_support"), "config", "prbt", "master.dcf"])
    can_interface = LaunchConfiguration("can_interface")

    master_bin_path = os.path.join(
                get_package_share_directory("prbt_robot_support"),
                "config",
                "prbt",
                "master.bin",
            )     
    if not os.path.exists(master_bin_path):
        master_bin_path = ""     
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(LaunchConfiguration("description_package")), "urdf", "prbt.xacro"]),
            " ",
            "bus_config:=",
            bus_config,
            " ",
            "master_config:=",
            master_config,
            " ",
            "master_bin:=",
            master_bin_path,
            " ",
            "can_interface:=",
            can_interface,
            " ",
            "use_ros2_control:=",
            LaunchConfiguration("use_ros2_control"),
            " ", 
        ]
    )
    robot_description = {"robot_description": launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Controller manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, controller_config],
    )

    controller_spawner_launch_file = PathJoinSubstitution([FindPackageShare("prbt_robot_support"), "launch", "prbt_controller_spawner.launch.py"])
    controller_spawner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                controller_spawner_launch_file,
            ]
        ),
        launch_arguments={}.items(),
    )

    nodes_list = [
        robot_state_publisher_node,
        controller_manager_node,
        controller_spawner_node,
    ]

    return LaunchDescription(declared_arguments + nodes_list)