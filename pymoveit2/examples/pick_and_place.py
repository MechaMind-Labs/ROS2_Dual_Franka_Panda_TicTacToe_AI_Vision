#!/usr/bin/env python3

"""
Pick and Place Mode:
ros2 run pymoveit2 pick_and_place.py --ros-args -p arm_id:=panda_1 -p mode:=pick_place

Tic-Tac-Toe Hover Mode:
ros2 run pymoveit2 pick_and_place.py --ros-args -p arm_id:=panda_1 -p mode:=hover

Default (pick_place):
ros2 run pymoveit2 pick_and_place.py --ros-args -p arm_id:=panda_1 -p box_num:="0" -p obj_num:="0"
"""

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2, GripperInterface


class CombinedRobotController(Node):
    def __init__(self):
        super().__init__("combined_robot_controller")

        # --- Parameters ---
        self.declare_parameter("arm_id", "panda_1")  # "panda_1" or "panda_2"
        self.declare_parameter("mode", "pick_place")  # "pick_place" or "hover"
        self.declare_parameter("box_num", 0)  # box1 [0] - box9 [8]
        self.declare_parameter("obj_num", 0)  # obj_1 [0] - obj_5 [4]
        
        self.arm_id = self.get_parameter("arm_id").value
        self.mode = self.get_parameter("mode").value
        self.box_num = self.get_parameter("box_num").value
        self.obj_num = self.get_parameter("obj_num").value

        # --- Validate mode ---
        if self.mode not in ["pick_place", "hover"]:
            self.get_logger().error(f"❌ Invalid mode: {self.mode}. Use 'pick_place' or 'hover'")
            return

        # --- Move group and gripper setup ---
        group_name = "arm_1" if self.arm_id == "panda_1" else "arm_2"
        gripper_group = "gripper_1" if self.arm_id == "panda_1" else "gripper_2"

        self.get_logger().info(f"🎯 Controlling {self.arm_id} | Mode: {self.mode} | MoveIt group: {group_name}")

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
            gripper_command_action_name=f"{self.arm_id}_gripper_controller/follow_joint_trajectory",
        )

        # --- Define joint configurations ---
        self.start_posn = [0.0, 0.0, 0.0, 0.0, 0.0, 1.572, 0.8]
        self.home_posn = [0.0, 0.0, 0.0, -1.572, 0.0, 1.572, 0.893]

        self.boxes = [
            [0.252, 0.448, -0.047, -2.033, 0.0, 2.461, -0.517],    # Box 1
            [0.016, 0.402, -0.047, -2.078, -0.016, 2.461, -0.736], # Box 2
            [-0.15, 0.39, -0.047, -2.056, -0.016, 2.443, -0.956],  # Box 3
            [0.033, 0.792, 0.167, -1.482, -0.172, 2.237, -0.487],  # Box 4
            [0.052, 0.758, -0.059, -1.510, 0.016, 2.239, -0.764],  # Box 5
            [-0.123, 0.772, -0.047, -1.498, 0.016, 2.265, -0.867], # Box 6
            [0.144, 1.541, 0.170, -0.07, -0.203, 1.734, 0.909],    # Box 7
            [0.01, 1.541, 0.17, -0.07, -0.203, 1.688, 0.909],      # Box 8
            [-0.158, 1.541, 0.17, -0.07, -0.163, 1.744, 0.670]     # Box 9
        ]

        self.objects = [
            [-2.240, 0.42, 0.486, -2.244, -0.110, 2.448, 0.987],     #obj_1
            [-1.364, 0.365, 0.00, -2.30, 0.00, 2.565, 1.079],        #obj_2
            [-1.081, 0.605, 0.035, -1.839, 0.047, 2.163, 1.362],     #obj_3
            [-1.457, -0.013, 0.078, -3.072, -0.329, 3.243, 1.362],   #obj_4
            [-0.615, 0.186, -0.141, -2.578, -0.203, 2.598, 1.709]    #obj_5
        ]

        self.obj_turns = [
            [-1.60, 0.0, 0.0, -1.572, 0.0, 1.572, 0.893], #obj_1
            [-1.70, 0.0, 0.0, -1.572, 0.0, 1.572, 0.893],  #obj_2
            [-0.7, 0.0, 0.0, -1.572, 0.0, 1.572, 0.893],  #obj_3
            [-0.8, 0.0, 0.0, -1.572, 0.0, 1.572, 0.893],  #obj_4
            [-0.7, 0.0, 0.0, -1.572, 0.0, 1.572, 0.893]   #obj_5
        ]

        self.bend_ups = [
            [-2.240, 0.0, 0.486, -2.244, -0.110, 2.448, 0.987],     #obj_1
            [-1.364, -0.124, 0.00, -2.30, 0.00, 2.565, 1.079],      #obj_2
            [-1.081, -0.086, 0.035, -1.839, 0.047, 1.796, 1.362],   #obj_3
            [-1.457, -0.696, 0.078, -3.072, -0.203, 2.550, 1.143],  #obj_4
            [-0.615, -0.295, -0.141, -2.578, -0.203, 2.306, 1.709]  #obj_5

        ]

        # For panda_2, reverse the order of boxes (Box1<->Box9, Box2<->Box8, etc.)
        if self.arm_id == "panda_2":
            self.boxes.reverse()

        # --- Run the selected sequence ---
        if self.mode == "pick_place":
            self.run_pick_place_sequence()
        elif self.mode == "hover":
            self.run_hover_sequence()

    def run_pick_place_sequence(self):
        """Pick and place sequence - picks object and places in first box"""
        log = self.get_logger().info

        log("🏁 Moving to Start position...")
        self.move_to(self.start_posn)

        log("🎯 Moving to Home position...")
        self.move_to(self.home_posn)

        log("🏁 Moving to turn position...")
        self.move_to(self.obj_turns[self.obj_num])

        log("🤲 Opening gripper...")
        self.gripper.open()
        self.gripper.wait_until_executed()

        log("📸 Moving to Object position...")
        self.move_to(self.objects[self.obj_num])

        log("✊ Closing gripper (grasping object)...")
        self.gripper.close()
        self.gripper.wait_until_executed()

        log("📸 Moving to bend_up position...")
        self.move_to(self.bend_ups[self.obj_num])

        log("🔙 Returning to Home position...")
        self.move_to(self.home_posn)

        log("⬜ Moving to BOX position...")
        self.move_to(self.boxes[self.box_num])

        log("🤲 Opening gripper (releasing object)...")
        self.gripper.open()
        self.gripper.wait_until_executed()

        log("🔙 Returning to Home position...")
        self.move_to(self.home_posn)

        log("✊ Closing gripper at home (reset)...")
        self.gripper.close()
        self.gripper.wait_until_executed()

        log("🏠 Moving back to Start position...")
        self.move_to(self.start_posn)

        log("✅ Pick and Place sequence complete!")

    def run_hover_sequence(self):
        """Tic-Tac-Toe hover sequence - hovers over all 9 boxes"""
        log = self.get_logger().info

        log("🏁 Moving to home position...")
        self.move_to(self.start_posn)

        log("🟦 Closing gripper...")
        self.gripper.close()
        self.gripper.wait_until_executed()

        # Hover over each box one by one
        for i, box in enumerate(self.boxes, start=1):
            log(f"📍 Moving to Box {i} joint position...")
            self.move_to(box)

        log("🏠 Returning to home position...")
        self.move_to(self.home_posn)
        self.move_to(self.start_posn)

        log("✅ Tic-Tac-Toe hover sequence complete!")

    def move_to(self, joints):
        """Move to joint configuration and wait"""
        self.moveit2.move_to_configuration(joints)
        self.moveit2.wait_until_executed()


def main():
    rclpy.init()
    node = CombinedRobotController()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()