#!/usr/bin/env python3

'''
ros2 run pymoveit2 testing.py --ros-args -p arm_id:=panda_1
ros2 run pymoveit2 testing.py --ros-args -p arm_id:=panda_2


'''

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2, GripperInterface
import math

class TicTacToeHover(Node):
    def __init__(self):
        super().__init__("tic_tac_toe_hover")

        # --- Choose which arm to control ---
        self.declare_parameter("arm_id", "panda_1")  # "panda_1" or "panda_2"
        self.arm_id = self.get_parameter("arm_id").value

        # --- Group names depend on which robot ---
        if self.arm_id == "panda_1":
            group_name = "arm_1"
            gripper_group = "gripper_1"
        else:
            group_name = "arm_2"
            gripper_group = "gripper_2"

        self.get_logger().info(f"🎯 Controlling {self.arm_id} | MoveIt group: {group_name}")

        # --- Arm interface ---
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[f"{self.arm_id}_joint{i}" for i in range(1, 8)],
            base_link_name=f"{self.arm_id}_link0",
            end_effector_name=f"{self.arm_id}_hand",
            group_name=group_name,
        )

        # --- Gripper interface ---
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=[
                f"{self.arm_id}_finger_joint1",
                f"{self.arm_id}_finger_joint2",
            ],
            open_gripper_joint_positions=[0.04, 0.04],
            closed_gripper_joint_positions=[0.0, 0.0],
            gripper_group_name=gripper_group,
            gripper_command_action_name=f"{self.arm_id}_gripper_controller/follow_joint_trajectory"
        )

        # --- Joint configurations ---
        self.home = [0.0, 0.0, 0.0, 0.0, 0.0, 1.572, 0.8]
        self.pick = [0.0, 0.0, 0.0, -1.572, 0.0, 1.572, 0.893]

        self.boxes = [
            [0.252, 0.448, -0.047, -2.033, 0.0, 2.461, -0.517],  # Box 1
            [0.016, 0.402, -0.047, -2.078, -0.016, 2.461, -0.736],
            [-0.15, 0.39, -0.047, -2.056, -0.016, 2.443, -0.956],
            [0.033, 0.792, 0.167, -1.482, -0.172, 2.237, -0.487],
            [0.052, 0.758, -0.059, -1.510, 0.016, 2.239, -0.764],
            [-0.123, 0.772, -0.047, -1.498, 0.016, 2.265, -0.867],
            [0.144, 1.541, 0.170, -0.07, -0.203, 1.734, 0.909],
            [0.01, 1.541, 0.17, -0.07, -0.203, 1.688, 0.909],
            [-0.158, 1.541, 0.17, -0.07, -0.163, 1.744, 0.670] #Box 9
        ]

        # For panda_2, reverse the order of boxes (Box1<->Box9, Box2<->Box8, etc.)
        if self.arm_id == "panda_2":
            self.boxes.reverse()

        self.run_sequence()

    def run_sequence(self):
        self.get_logger().info("🏁 Moving to home position...")
        self.move_to(self.home)

        self.get_logger().info("🟦 Closing gripper...")
        self.gripper.close()
        self.gripper.wait_until_executed()

        # Hover over each box one by one
        for i, box in enumerate(self.boxes, start=1):
            self.get_logger().info(f"📍 Moving to Box {i} joint position...")
            self.move_to(box)

        self.get_logger().info("🏠 Returning to home position...")
        self.move_to(self.pick)
        self.move_to(self.home)

        self.get_logger().info("✅ Tic-Tac-Toe hover sequence complete!")

    def move_to(self, joints):
        self.moveit2.move_to_configuration(joints)
        self.moveit2.wait_until_executed()


def main():
    rclpy.init()
    node = TicTacToeHover()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
