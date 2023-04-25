import os
import launch_ros
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    slave_eds_path = DeclareLaunchArgument(
        'eds_path',
        default_value=TextSubstitution(text=""),
        description="Path to eds file to be used for the slave.",
    )
    
    ld.add_action(slave_eds_path)

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
                "slave_config": LaunchConfiguration('eds_path'),
                }.items(),
        )
        ld.add_action(slave_node)
    
    return ld
