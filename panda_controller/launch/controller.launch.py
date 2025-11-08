import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# This launch file is designed to be included from your main launch file
# It assumes that the 'ign_ros2_control' plugin in your URDF has already
# loaded the 'multi_panda_controllers.yaml' file.
#
# This file's only job is to "spawn" (i.e., start) the controllers
# that were configured by that YAML file.

def generate_launch_description():

    # A list of all controllers defined in your YAML file
    # We will create one "spawner" node for each of them
    controllers_to_spawn = [
        "joint_state_broadcaster",
        "panda_1_arm_controller",
        "panda_1_gripper_controller",
        "panda_2_arm_controller",
        "panda_2_gripper_controller",
    ]

    # Create a list of spawner nodes, one for each controller
    spawner_nodes = []
    for controller in controllers_to_spawn:
        spawner_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,                      # The name of the controller to spawn
                    "-c", "/controller_manager"      # The namespace of the C.M. service
                ],
            )
        )

    # The joint_state_broadcaster is commented out in your YAML,
    # but I've kept the logic here in case you re-enable it.
    # It's good practice to spawn the joint_state_broadcaster first.
    #
    # NOTE: Your new YAML file uses *one* 'joint_state_broadcaster'
    # for both robots, which is perfectly fine. It will find all
    # available state interfaces and broadcast them.

    return LaunchDescription(spawner_nodes)
