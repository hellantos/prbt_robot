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

    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "3",
            "node_name": "prbt_node_1",
            "slave_config": slave_config,
        }.items(),
        condition=IfCondition(LaunchConfiguration("fake_drives")),
    )
    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "4",
            "node_name": "prbt_node_2",
            "slave_config": slave_config,
        }.items(),
        condition=IfCondition(LaunchConfiguration("fake_drives")),
    )
    slave_node_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "5",
            "node_name": "prbt_node_3",
            "slave_config": slave_config,
        }.items(),
        condition=IfCondition(LaunchConfiguration("fake_drives")),
    )
    slave_node_4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "6",
            "node_name": "prbt_node_4",
            "slave_config": slave_config,
        }.items(),
        condition=IfCondition(LaunchConfiguration("fake_drives")),
    )
    slave_node_5 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "7",
            "node_name": "prbt_node_5",
            "slave_config": slave_config,
        }.items(),
        condition=IfCondition(LaunchConfiguration("fake_drives")),
    )
    slave_node_6 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "8",
            "node_name": "prbt_node_6",
            "slave_config": slave_config,
        }.items(),
        condition=IfCondition(LaunchConfiguration("fake_drives")),
    )

    virtual_joints = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
            )
        ),
    )

    # Given the published joint states, publish tf for the robot links

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/rsp.launch.py")
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

    # If database loading was enabled, start mongodb as well
    db = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/warehouse_db.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("db")),
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
    )

    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
        ),
    )

    nodes_to_start = [
        fake_drives_arg,
        debug_arg,
        db_arg,
        use_rviz_arg,
        slave_node_1,
        slave_node_2,
        slave_node_3,
        slave_node_4,
        slave_node_5,
        slave_node_6,
        TimerAction(
            period=2.0,
            actions=[
                controller_manager,
            ],
        ),
        TimerAction(
            period=4.0,
            actions=[
                virtual_joints,
                robot_state_publisher,
                move_group,
                db,
            ]
        ),
        TimerAction(
            period=6.0,
            actions=[spawn_controllers]
        ),
        TimerAction(
            period=8.0,
            actions=[rviz]
        )
    ]

    return LaunchDescription(nodes_to_start)
