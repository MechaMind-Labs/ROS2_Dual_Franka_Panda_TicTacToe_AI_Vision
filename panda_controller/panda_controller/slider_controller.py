#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SliderController(Node):
    def __init__(self):
        super().__init__("slider_controller")

        # --- Publishers for both Panda arms and grippers ---
        self.p1_arm_pub = self.create_publisher(JointTrajectory, "panda_1_arm_controller/joint_trajectory", 10)
        self.p1_gripper_pub = self.create_publisher(JointTrajectory, "panda_1_gripper_controller/joint_trajectory", 10)
        self.p2_arm_pub = self.create_publisher(JointTrajectory, "panda_2_arm_controller/joint_trajectory", 10)
        self.p2_gripper_pub = self.create_publisher(JointTrajectory, "panda_2_gripper_controller/joint_trajectory", 10)

        # Expected joint names (exact strings controllers expect)
        self.p1_arm_joints = [
            "panda_1_joint1", "panda_1_joint2", "panda_1_joint3",
            "panda_1_joint4", "panda_1_joint5", "panda_1_joint6", "panda_1_joint7"
        ]
        self.p1_gripper_joints = ["panda_1_finger_joint1", "panda_1_finger_joint2"]

        self.p2_arm_joints = [
            "panda_2_joint1", "panda_2_joint2", "panda_2_joint3",
            "panda_2_joint4", "panda_2_joint5", "panda_2_joint6", "panda_2_joint7"
        ]
        self.p2_gripper_joints = ["panda_2_finger_joint1", "panda_2_finger_joint2"]

        # Subscribe to slider joint topic
        self.sub = self.create_subscription(JointState, "joint_commands", self.slider_callback, 10)

        self.get_logger().info("✅ Slider Controller Node started (name-mapped mode)")

    # def _build_pos_map(self, msg: JointState):
    #     """Return dict: joint_name -> position. If name list empty, try index-based fallback."""
    #     pos_map = {}
    #     if msg.name and len(msg.name) == len(msg.position):
    #         pos_map = {n: p for n, p in zip(msg.name, msg.position)}
    #     else:
    #         # Fallback: if no names provided, we cannot map reliably.
    #         self.get_logger().debug("Incoming JointState has no names or mismatch; cannot map by name.")
    #     return pos_map

    def _build_pos_map(self, msg: JointState):
        """Return dict: joint_name -> position. If name list empty, try index-based fallback."""
        pos_map = {}
        if msg.name and len(msg.name) == len(msg.position):
            pos_map = {n: p for n, p in zip(msg.name, msg.position)}

            # --- FIX START ---
            # Alias handling for joint2 names if GUI/slider sends them without prefixes
            if "joint2" in pos_map:
                # Copy same position value to both panda_1 and panda_2 joint2
                pos_map["panda_1_joint2"] = pos_map["joint2"]
                pos_map["panda_2_joint2"] = pos_map["joint2"]
            # --- FIX END ---

        else:
            self.get_logger().debug("Incoming JointState has no names or mismatch; cannot map by name.")
        return pos_map


    def _all_present(self, required, pos_map):
        """Return True if all required joint names are present in pos_map."""
        return all(name in pos_map for name in required)

    def slider_callback(self, msg: JointState):
        """Map incoming JointState by name and publish trajectories for controllers that have full data."""
        pos_map = self._build_pos_map(msg)

        if not pos_map:
            self.get_logger().warn("No joint names in incoming JointState — please publish names or fix joint_state_publisher GUI.")
            return

        # --- Panda 1 ---
        if self._all_present(self.p1_arm_joints, pos_map):
            p1_arm_msg = JointTrajectory()
            p1_arm_msg.joint_names = self.p1_arm_joints
            p1_pt = JointTrajectoryPoint()
            p1_pt.positions = [pos_map[name] for name in self.p1_arm_joints]
            p1_arm_msg.points.append(p1_pt)
            self.p1_arm_pub.publish(p1_arm_msg)
        else:
            missing = [n for n in self.p1_arm_joints if n not in pos_map]
            self.get_logger().warn(f"Skipping panda_1_arm publish — missing joints: {missing}")

        if self._all_present(self.p1_gripper_joints, pos_map):
            p1_grip_msg = JointTrajectory()
            p1_grip_msg.joint_names = self.p1_gripper_joints
            # send same value to both fingers (mimic)
            val = pos_map[self.p1_gripper_joints[0]]
            p1_grip_pt = JointTrajectoryPoint()
            p1_grip_pt.positions = [val, val]
            p1_grip_msg.points.append(p1_grip_pt)
            self.p1_gripper_pub.publish(p1_grip_msg)
        else:
            missing = [n for n in self.p1_gripper_joints if n not in pos_map]
            self.get_logger().warn(f"Skipping panda_1_gripper publish — missing joints: {missing}")

        # --- Panda 2 ---
        if self._all_present(self.p2_arm_joints, pos_map):
            p2_arm_msg = JointTrajectory()
            p2_arm_msg.joint_names = self.p2_arm_joints
            p2_pt = JointTrajectoryPoint()
            p2_pt.positions = [pos_map[name] for name in self.p2_arm_joints]
            p2_arm_msg.points.append(p2_pt)
            self.p2_arm_pub.publish(p2_arm_msg)
        else:
            missing = [n for n in self.p2_arm_joints if n not in pos_map]
            self.get_logger().warn(f"Skipping panda_2_arm publish — missing joints: {missing}")

        if self._all_present(self.p2_gripper_joints, pos_map):
            p2_grip_msg = JointTrajectory()
            p2_grip_msg.joint_names = self.p2_gripper_joints
            val = pos_map[self.p2_gripper_joints[0]]
            p2_grip_pt = JointTrajectoryPoint()
            p2_grip_pt.positions = [val, val]
            p2_grip_msg.points.append(p2_grip_pt)
            self.p2_gripper_pub.publish(p2_grip_msg)
        else:
            missing = [n for n in self.p2_gripper_joints if n not in pos_map]
            self.get_logger().warn(f"Skipping panda_2_gripper publish — missing joints: {missing}")

        self.get_logger().debug("Processed joint_commands message.")

def main():
    rclpy.init()
    node = SliderController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
