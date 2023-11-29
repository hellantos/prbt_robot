from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
)
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "prbt", package_name="prbt_robot_moveit_config"
    ).to_moveit_configs()

    slave_config = PathJoinSubstitution(
        [FindPackageShare("prbt_robot_support"), "config/prbt", "prbt_0_1.dcf"]
    )

    slave_launch = PathJoinSubstitution(
        [FindPackageShare("canopen_fake_slaves"), "launch", "cia402_slave.launch.py"]
    )

    fake_drives_arg = DeclareBooleanLaunchArg(
        "fake_drives",
        default_value=True,
        description="By default, we use fake hardware.",
    )

    db_arg = DeclareBooleanLaunchArg(
        "db",
        default_value=False,
        description="By default, we do not start a database (it can be large)",
    )

    debug_arg = DeclareBooleanLaunchArg(
        "debug",
        default_value=False,
        description="By default, we are not in debug mode",
    )

    use_rviz_arg = DeclareBooleanLaunchArg("use_rviz", default_value=True)

    virtual_joints = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
            )
        ),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/move_group.launch.py")
        ),
    )

    # Run Rviz and load the default config to see the state of the move_group node

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    nodes_to_start = [
        fake_drives_arg,
        debug_arg,
        db_arg,
        use_rviz_arg,
        virtual_joints,
        move_group,
        TimerAction(
            period=8.0,
            actions=[rviz]
        )
    ]

    return LaunchDescription(nodes_to_start)
