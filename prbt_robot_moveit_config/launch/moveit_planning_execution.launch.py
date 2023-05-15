import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros

from launch.actions import TimerAction
from launch.conditions import IfCondition

from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
)
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "prbt", package_name="prbt_robot_moveit_config"
    ).to_moveit_configs()
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="vcan0",
            description="Interface name for can",
        )
    )
    
    robot_hw_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("prbt_robot_support"), "launch", "robot.launch.py"])],
        ),
        launch_arguments={
            "can_interface": LaunchConfiguration("can_interface"),
            "use_ros2_control": "true",
        }.items(),
    )

    # todo: remove this once joint_state_broadcast from controller is fixed
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

    virtual_joints = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
            )
        ),
    )

    # Given the published joint states, publish tf for the robot links

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/move_group.launch.py")
        ),
    )

    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
        ),
    )

    # Run Rviz and load the default config to see the state of the move_group node

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
        ),
    )

    # If database loading was enabled, start mongodb as well
    db = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/warehouse_db.launch.py")
        ),
    )

    node_list = [
        robot_hw_node,
        #state_publisher,
        TimerAction(
            period=10.0,
            actions=[virtual_joints]
        ),
        # TimerAction(
        #     period=12.0,
        #     actions=[spawn_controllers]
        # ),
        TimerAction(
            period=15.0,
            actions=[move_group]
        ),
        TimerAction(
            period=20.0,
            actions=[rviz]
        )
    ]

    return LaunchDescription(declared_arguments + node_list)

   