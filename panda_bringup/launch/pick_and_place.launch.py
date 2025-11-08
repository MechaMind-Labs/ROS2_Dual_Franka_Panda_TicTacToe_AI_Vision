import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ------------------- Gazebo -------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
    )

    # ------------------- Controllers -------------------
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    slider_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_controller"),
                "launch",
                "slider_controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- MoveIt -------------------
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_moveit"),
                "launch",
                "moveit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- Vision Node -------------------
    vision_mapper_node = Node(
        package="panda_vision",
        executable="tic_tac_toe_mapper",
        name="color_detector",
        output="screen"
    )

    vision_winner_node = Node(
        package="panda_vision",
        executable="tic_tac_toe_winner",
        name="winner_detector",
        output="screen"
    )

    # ------------------- Blue Controller Node -------------------
    blue_controller_node = Node(
        package="pymoveit2",
        executable="tic_tac_toe_f1_blue.py",
        name="Blue_Controller",
        output="screen"
    )

    # ------------------- Green Controller Node -------------------
    green_controller_node = Node(
        package="pymoveit2",
        executable="tic_tac_toe_f2_green.py",
        name="Green_Controller",
        output="screen"
    )

    # ------------------- AI Node -------------------
    ai_decision_node = Node(
        package="panda_ai",
        executable="decision_maker",
        name="AI_Decision_Node",
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        controller,
        # slider_controller,
        moveit,
        blue_controller_node,
        green_controller_node,
        vision_mapper_node,
        vision_winner_node,
        ai_decision_node,
    ])
