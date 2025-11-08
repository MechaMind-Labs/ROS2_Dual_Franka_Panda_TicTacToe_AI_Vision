#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool

class TicTacToeWinner(Node):
    def __init__(self):
        super().__init__('tic_tac_toe_winner')

        self.sub = self.create_subscription(Int32MultiArray, '/grid_update', self.update_callback, 10)
        self.win_pub = self.create_publisher(Bool, '/game_over', 10)

        self.get_logger().info("🏁 Tic Tac Toe Winner Detector Started (Mapping 0|3|6, 1|4|7, 2|5|8)")

        self.grid = [0] * 9  # 0=empty, 1=Blue, 2=Green
        self.size = 500
        self.step = self.size // 3
        self.board = np.ones((self.size, self.size, 3), dtype=np.uint8) * 255
        self.draw_grid()

        self.winner = 0
        self.combo = []
        self.game_over = False  # Flag to avoid repeating win messages

        cv2.namedWindow("Winner Board")
        self.timer = self.create_timer(0.05, self.display_board)
        self.timer_pub = self.create_timer(0.1, self.publish_game_status)

    def draw_grid(self):
        """Draw base 3x3 grid"""
        self.board[:] = 255
        for i in range(1, 3):
            cv2.line(self.board, (i * self.step, 0), (i * self.step, self.size), (0, 0, 0), 4)
            cv2.line(self.board, (0, i * self.step), (self.size, i * self.step), (0, 0, 0), 4)

    def update_callback(self, msg):
        """Receive grid update and check for a winner"""
        if self.game_over:
            return  # Stop further updates after win or tie

        data = list(msg.data)
        if len(data) < 2:
            self.get_logger().warn("⚠️ Invalid message received on /grid_update")
            return

        grid_num, color = data
        if not (0 <= grid_num < 9) or color not in [1, 2] or self.grid[grid_num] != 0:
            return

        self.grid[grid_num] = color
        self.draw_grid()
        self.draw_moves()

        # --- Check for winner ---
        self.winner, self.combo = self.check_winner()
        if self.winner != 0:
            self.draw_winning_line(self.combo)
            text = "BLUE/Human Won!" if self.winner == 1 else "GREEN/AI Won!"
            self.get_logger().info(f"🎉 {text}")
            cv2.putText(self.board, text, (70, self.size - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 3)
            self.game_over = True
            return

        # --- Check for tie (no empty cells and no winner) ---
        if all(cell != 0 for cell in self.grid):
            text = "Game TIED!"
            self.get_logger().info(text)
            cv2.putText(self.board, text, (120, self.size - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (80, 80, 80), 3)
            self.game_over = True

    def publish_game_status(self):
        """Continuously publish game status True/False"""
        msg = Bool()
        msg.data = self.game_over
        self.win_pub.publish(msg)

    def index_to_coords(self, idx):
        """Column-wise mapping (0|3|6, 1|4|7, 2|5|8)"""
        col = idx // 3
        row = idx % 3
        return row, col

    def draw_moves(self):
        """Draw circles for all moves"""
        for idx, color in enumerate(self.grid):
            if color == 0:
                continue
            row, col = self.index_to_coords(idx)
            cx = int(col * self.step + self.step / 2)
            cy = int(row * self.step + self.step / 2)
            col_bgr = (255, 0, 0) if color == 1 else (0, 255, 0)
            cv2.circle(self.board, (cx, cy), self.step // 5, col_bgr, -1)

    def check_winner(self):
        """Return (winner_color, winning_combination)"""
        wins = [
            [0, 3, 6], [1, 4, 7], [2, 5, 8],  # columns
            [0, 1, 2], [3, 4, 5], [6, 7, 8],  # rows
            [0, 4, 8], [2, 4, 6]              # diagonals
        ]
        for combo in wins:
            a, b, c = combo
            if self.grid[a] != 0 and self.grid[a] == self.grid[b] == self.grid[c]:
                return self.grid[a], combo
        return 0, []

    def draw_winning_line(self, combo):
        """Draw thick red line across the winning combination"""
        first, _, last = combo
        row1, col1 = self.index_to_coords(first)
        row2, col2 = self.index_to_coords(last)
        p1 = (int(col1 * self.step + self.step / 2),
              int(row1 * self.step + self.step / 2))
        p2 = (int(col2 * self.step + self.step / 2),
              int(row2 * self.step + self.step / 2))
        cv2.line(self.board, p1, p2, (0, 0, 255), 12)

    def display_board(self):
        """Periodic OpenCV display (non-blocking)"""
        cv2.imshow("Winner Board", self.board)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TicTacToeWinner()
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
