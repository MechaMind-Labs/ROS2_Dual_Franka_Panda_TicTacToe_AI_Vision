#!/usr/bin/env python3
"""
ros2 run pymoveit2 tic_tac_toe_f2_green.py 
"""
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2, GripperInterface
from std_msgs.msg import String, Int32, Bool
import threading


class GreenRobotController(Node):
    def __init__(self):
        super().__init__("green_robot_controller")
        
        # --- Subscribers ---
        self.create_subscription(String, "/player_turn", self.turn_callback, 10)
        self.create_subscription(Int32, "/grid_posn", self.posn_callback, 10)
        
        # --- Publisher ---
        self.is_moving_pub = self.create_publisher(Bool, '/is_moving_green', 10)
        
        self.latest_turn = None
        self.latest_box = None
        self.is_moving = False  # Flag to prevent simultaneous movements
        
        # Publish initial state (not moving)
        self.publish_moving_status(False)
        
        self.move_lock = threading.Lock()  # Thread safety
        self.get_logger().info("🟩 Panda 2 (Green) controller ready!")
        
        # --- Initialize MoveIt2 + Gripper ---
        self.arm_id = "panda_2"
        self.color = "green"
        group_name = "arm_2"
        gripper_group = "gripper_2"
        
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

        self.boxes.reverse()
        
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
        
        self.obj_turn = 0
    
    def publish_moving_status(self, is_moving):
        """Publish movement status"""
        msg = Bool()
        msg.data = is_moving
        self.is_moving_pub.publish(msg)
        self.get_logger().info(f"📡 Green moving status: {is_moving}")
    
    def turn_callback(self, msg):
        """Receives player turn updates"""
        with self.move_lock:
            self.latest_turn = msg.data.strip().lower()
            self.get_logger().info(f"🔄 Turn received: {self.latest_turn}")
    
    def posn_callback(self, msg):
        """Receives grid position and executes move if it's Green's turn"""
        # Use lock to prevent race conditions
        with self.move_lock:
            self.latest_box = msg.data
            self.get_logger().info(f"📍 Position received: {self.latest_box}, Current turn: {self.latest_turn}")
            
            # CRITICAL: Check if it's green's turn first - if not, ignore completely
            if self.latest_turn != self.color:
                self.get_logger().info(f"⏸ Not my turn ({self.latest_turn}), ignoring position")
                return
            
            # Validate inputs
            if self.latest_box < 0 or self.latest_box >= len(self.boxes):
                self.get_logger().error(f"❌ Invalid box number: {self.latest_box}")
                return
            
            if self.obj_turn >= len(self.objects):
                self.get_logger().error(f"❌ No more objects available!")
                return
            
            if self.is_moving:
                self.get_logger().warn(f"⚠️ Robot is already moving, ignoring command")
                return
            
            # It's my turn, proceed with movement
            self.get_logger().info(f"🎮 GREEN EXECUTING! Box {self.latest_box}, Object {self.obj_turn}")
            
            # Run in separate thread to avoid blocking callbacks
            move_thread = threading.Thread(target=self._execute_move)
            move_thread.daemon = True
            move_thread.start()
    
    def _execute_move(self):
        """Execute the pick and place in a separate thread"""
        self.is_moving = True
        self.publish_moving_status(True)
        
        try:
            self.run_pick_place_sequence()
            self.obj_turn += 1
            self.get_logger().info(f"✅ Green move complete! Next object: {self.obj_turn}")
        except Exception as e:
            self.get_logger().error(f"❌ Error during movement: {e}")
        finally:
            self.is_moving = False
            self.publish_moving_status(False)
    
    def run_pick_place_sequence(self):
        """Pick and place sequence - picks object and places in box"""
        log = self.get_logger().info
        
        log("🏁 Moving to Start position...")
        self.move_to(self.start_posn)
        
        log("🎯 Moving to Home position...")
        self.move_to(self.home_posn)
        
        log("🏁 Moving to turn position...")
        self.move_to(self.obj_turns[self.obj_turn])
        
        log("🤲 Opening gripper...")
        self.gripper.open()
        self.gripper.wait_until_executed()
        
        log("📸 Moving to Object position...")
        self.move_to(self.objects[self.obj_turn])
        
        log("✊ Closing gripper (grasping object)...")
        self.gripper.close()
        self.gripper.wait_until_executed()
        
        log("📸 Moving to bend_up position...")
        self.move_to(self.bend_ups[self.obj_turn])
        
        log("🔙 Returning to Home position...")
        self.move_to(self.home_posn)
        
        log("⬜ Moving to BOX position...")
        self.move_to(self.boxes[self.latest_box])
        
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
    
    def move_to(self, joints):
        """Move to joint configuration and wait"""
        self.moveit2.move_to_configuration(joints)
        self.moveit2.wait_until_executed()


def main():
    rclpy.init()
    node = GreenRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()