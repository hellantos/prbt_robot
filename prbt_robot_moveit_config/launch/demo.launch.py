import pprint
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("prbt", package_name="prbt_robot_moveit_config").to_moveit_configs()
    pprint.pprint(moveit_config.to_dict())
    return generate_demo_launch(moveit_config)
