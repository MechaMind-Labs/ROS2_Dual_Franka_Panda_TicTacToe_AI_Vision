#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32, String, Bool
from std_msgs.msg import Int32MultiArray
import time


class TicTacToeMapper(Node):
    def __init__(self):
        super().__init__('tic_tac_toe_mapper')

        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # --- Publishers ---
        self.grid_pub = self.create_publisher(Int32, '/grid_posn', 10)
        self.turn_pub = self.create_publisher(String, '/player_turn', 10)
        self.grid_update_pub = self.create_publisher(Int32MultiArray, '/grid_update', 10)
        self.ai_ask_pub = self.create_publisher(Bool, '/ai_ask', 10)

        # --- Subscribers ---
        # Subscribe to robot movement status
        self.blue_is_moving_sub = self.create_subscription(Bool, '/is_moving_blue', self.blue_is_moving_callback, 10)
        self.is_moving_blue = False
        self.was_moving_blue = False

        self.green_is_moving_sub = self.create_subscription(Bool, '/is_moving_green', self.green_is_moving_callback, 10)
        self.is_moving_green = False
        self.was_moving_green = False

        self.game_over_sub = self.create_subscription(Bool, '/game_over', self.game_over_callback, 10)
        self.game_over = False


        # Subscribe to AI move decision
        self.ai_move_sub = self.create_subscription(Int32, '/ai_move', self.ai_move_callback, 10)

        self.get_logger().info("🎯 Tic Tac Toe Color Mapper Started")

        # HSV thresholds
        self.lower_hsv = np.array([25, 59, 0])
        self.upper_hsv = np.array([172, 280, 313])
        self.min_area = 300

        # Color ranges for detection
        self.color_ranges = {
            'B': ([90, 60, 60], [130, 255, 255]),   # Blue
            'G': ([40, 50, 50], [80, 255, 255])     # Green
        }

        # Game tracking
        self.step = None
        self.size = None
        self.current_turn = 'blue'  # Always starts with blue
        self.latest_grid_img = None
        self.last_click_time = 0
        self.click_cooldown = 0.5

        # Set up mouse click callback
        cv2.namedWindow("Tic Tac Toe Board")
        cv2.setMouseCallback("Tic Tac Toe Board", self.mouse_callback)

    def mouse_callback(self, event, x, y, flags, param):
        if self.game_over:
            self.get_logger().warn("🏁 Game over — no further human moves allowed!")
            return

        """Handle mouse click - only works during BLUE's turn"""
        if event == cv2.EVENT_LBUTTONDOWN and self.step is not None:
            # Only allow clicks during blue's turn
            if self.current_turn != 'blue':
                self.get_logger().warn("⚠️ Not blue's turn! Waiting for green (AI)...")
                return

            # Debounce
            current_time = time.time()
            if current_time - self.last_click_time < self.click_cooldown:
                self.get_logger().warn("⏳ Click too fast! Please wait...")
                return
            self.last_click_time = current_time

            grid_x = x // self.step
            grid_y = y // self.step

            # Safety bounds
            grid_x = min(max(grid_x, 0), 2)
            grid_y = min(max(grid_y, 0), 2)

            # Custom mapping
            mapping = [
                [0, 3, 6],
                [1, 4, 7],
                [2, 5, 8]
            ]
            grid_num = mapping[grid_y][grid_x]

            # Publish for blue player
            self.publish_move(grid_num, 'blue')

    def ai_move_callback(self, msg):
        if self.game_over:
            self.get_logger().warn("🏁 Game over — AI move skipped!")
            return

        """Receives AI's chosen move and publishes it"""
        if self.current_turn != 'green':
            self.get_logger().warn("⚠️ Received AI move but it's not green's turn!")
            return

        grid_num = msg.data
        self.get_logger().info(f"🤖 AI chose grid {grid_num}")
        
        # Publish for green player (AI)
        self.publish_move(grid_num, 'green')

    def publish_move(self, grid_num, player):
        """Publishes turn, grid position, and grid update"""
        # Publish turn FIRST
        turn_msg = String()
        turn_msg.data = player
        self.turn_pub.publish(turn_msg)
        self.get_logger().info(f"📢 Turn: {player.upper()}")
        
        # Small delay
        time.sleep(0.1)
        
        # Publish grid position
        msg = Int32()
        msg.data = grid_num
        self.grid_pub.publish(msg)
        
        # Publish grid + color (Blue=1, Green=2)
        grid_msg = Int32MultiArray()
        grid_msg.data = [grid_num, 1 if player == 'blue' else 2]
        self.grid_update_pub.publish(grid_msg)

        self.get_logger().info(f"🎮 Grid {grid_num} -> {player.upper()}")

    def image_callback(self, msg):
        """Process the incoming camera frame and draw Tic Tac Toe overlay"""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Crop to square
        h, w, _ = frame.shape
        side = min(h, w)
        start_x = (w - side) // 2
        start_y = (h - side) // 2
        frame = frame[start_y:start_y + side, start_x:start_x + side]
        self.size = side
        self.step = side // 3

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        positions = []

        for label, (lower, upper) in self.color_ranges.items():
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > self.min_area:
                    x, y, w_box, h_box = cv2.boundingRect(cnt)
                    cx, cy = x + w_box // 2, y + h_box // 2
                    cv2.rectangle(frame, (x, y), (x + w_box, y + h_box), (0, 255, 255), 2)
                    cv2.putText(frame, label, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    positions.append((cx, cy, label))

        # Draw Tic Tac Toe grid
        grid_img = self.draw_tic_tac_toe(side, positions)
        self.latest_grid_img = grid_img

        # Show next turn on board
        if self.current_turn == 'blue':
            turn_text = "Next: BLUE/Human"
            turn_color = (255, 0, 0)
        else:
            turn_text = "Next: GREEN/AI"
            turn_color = (0, 255, 0)

        cv2.putText(grid_img, turn_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, turn_color, 2)

        cv2.imshow("Detected Frame", frame)
        cv2.imshow("Tic Tac Toe Board", grid_img)
        cv2.waitKey(1)

    def draw_tic_tac_toe(self, size, positions):
        """Draws 3x3 grid and marks the detected color positions"""
        board = np.ones((size, size, 3), dtype=np.uint8) * 255
        step = size // 3

        # Draw grid lines
        for i in range(1, 3):
            cv2.line(board, (i * step, 0), (i * step, size), (0, 0, 0), 4)
            cv2.line(board, (0, i * step), (size, i * step), (0, 0, 0), 4)

        # Draw pieces
        for (cx, cy, label) in positions:
            grid_x = cx // step
            grid_y = cy // step
            center_x = int(grid_x * step + step / 2)
            center_y = int(grid_y * step + step / 2)
            color = (255, 0, 0) if label == 'B' else (0, 255, 0)
            cv2.circle(board, (center_x, center_y), step // 5, color, -1)

        return board
    
    def blue_is_moving_callback(self, msg):
        """Receives updates of Blue Robot State and toggles turn when movement completes."""
        self.is_moving_blue = msg.data
        
        # Detect transition from True to False (movement complete)
        if self.was_moving_blue and not self.is_moving_blue:
            self.get_logger().info("✅ Blue robot finished moving!")
            # Toggle turn to green and ask AI for move
            self.current_turn = 'green'
            self.get_logger().info(f"🔄 Turn toggled to: {self.current_turn.upper()}")
            
            # Ask AI to make a decision
            ai_ask_msg = Bool()
            ai_ask_msg.data = True
            self.ai_ask_pub.publish(ai_ask_msg)
            self.get_logger().info("🤖 Asking AI for next move...")
        
        # Update previous state
        self.was_moving_blue = self.is_moving_blue

    def green_is_moving_callback(self, msg):
        """Receives updates of Green Robot State and toggles turn when movement completes."""
        self.is_moving_green = msg.data
        
        # Detect transition from True to False (movement complete)
        if self.was_moving_green and not self.is_moving_green:
            self.get_logger().info("✅ Green robot finished moving!")
            # Toggle turn back to blue
            self.current_turn = 'blue'
            self.get_logger().info(f"🔄 Turn toggled to: {self.current_turn.upper()}")
            
            # Reset AI ask to False
            ai_ask_msg = Bool()
            ai_ask_msg.data = False
            self.ai_ask_pub.publish(ai_ask_msg)
            self.get_logger().info("🔴 AI ask set to False")
        
        # Update previous state
        self.was_moving_green = self.is_moving_green

    def game_over_callback(self, msg):
        self.game_over = msg.data
        if self.game_over:
            self.get_logger().warn("🏁 Game Over! No more moves allowed.")



def main(args=None):
    rclpy.init(args=args)
    node = TicTacToeMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()