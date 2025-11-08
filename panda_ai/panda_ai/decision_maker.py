#!/usr/bin/env python3
import os
import google.generativeai as genai
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, Bool


class DecisionMaker(Node):
    def __init__(self):
        super().__init__('decision_maker')
        
        # --- Gemini Setup ---
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            self.get_logger().error("❌ GEMINI_API_KEY not set in environment!")
            exit(1)
        
        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel('gemini-2.0-flash')
        
        # --- ROS Setup ---
        self.grid_update_sub = self.create_subscription(
            Int32MultiArray, '/grid_update', self.update_callback, 10)
        
        self.ai_ask_sub = self.create_subscription(
            Bool, '/ai_ask', self.ai_ask_callback, 10)
        
        self.grid_update_pub = self.create_publisher(Int32MultiArray, '/grid_update', 10)
        self.ai_move_pub = self.create_publisher(Int32, '/ai_move', 10)
        
        # --- Internal Grid State ---
        self.grid = [0] * 9
        self.get_logger().info("🧠 Decision Maker Node Started — Human(Blue=1) vs AI(Green=2)")
    

    # ==========================================================
    # 🧩 Callback Functions
    # ==========================================================
    def update_callback(self, msg):
        """Receives updates of the board and stores the state."""
        grid_num, color = msg.data
        self.grid[grid_num] = color
        
        if color == 1:
            self.get_logger().info(f"🟦 Human played at {grid_num}")
        elif color == 2:
            self.get_logger().info(f"🟩 AI move confirmed at {grid_num}")
    
    def ai_ask_callback(self, msg):
        """Triggered when mapper asks AI to make a move."""
        if msg.data:
            self.get_logger().info("🤖 AI asked to make a move...")
            ai_move = self.ask_gemini_for_move()
            
            if ai_move is not None:
                self.make_ai_move(ai_move)
    

    # ==========================================================
    # 🧮 Core Logic
    # ==========================================================
    def find_blocking_or_winning_move(self, grid, player, opponent):
        """Heuristic: check for immediate winning or blocking move."""
        win_patterns = [
            [0, 3, 6], [1, 4, 7], [2, 5, 8],  # columns
            [0, 1, 2], [3, 4, 5], [6, 7, 8],  # rows
            [0, 4, 8], [2, 4, 6]              # diagonals
        ]
        # Try to win first
        for a, b, c in win_patterns:
            vals = [grid[a], grid[b], grid[c]]
            if vals.count(player) == 2 and vals.count(0) == 1:
                return [a, b, c][vals.index(0)]
        # Try to block opponent
        for a, b, c in win_patterns:
            vals = [grid[a], grid[b], grid[c]]
            if vals.count(opponent) == 2 and vals.count(0) == 1:
                return [a, b, c][vals.index(0)]
        return None
    

    def is_better_move(self, grid, move_a, move_b, player, opponent):
        """Compare two moves: prefer winning or blocking moves."""
        if move_a == move_b or move_a is None:
            return False
        if self.check_winning_move(grid, move_a, player):
            return True
        if self.check_blocking_move(grid, move_a, opponent):
            return True
        return False


    def check_winning_move(self, grid, move, player):
        """Check if placing 'player' at 'move' wins the game."""
        temp_grid = grid.copy()
        temp_grid[move] = player
        win_patterns = [
            [0, 3, 6], [1, 4, 7], [2, 5, 8],
            [0, 1, 2], [3, 4, 5], [6, 7, 8],
            [0, 4, 8], [2, 4, 6]
        ]
        return any(all(temp_grid[i] == player for i in pattern) for pattern in win_patterns)


    def check_blocking_move(self, grid, move, opponent):
        """Check if placing here blocks opponent's win."""
        temp_grid = grid.copy()
        temp_grid[move] = 2  # AI plays here
        win_patterns = [
            [0, 3, 6], [1, 4, 7], [2, 5, 8],
            [0, 1, 2], [3, 4, 5], [6, 7, 8],
            [0, 4, 8], [2, 4, 6]
        ]
        for pattern in win_patterns:
            if all(temp_grid[i] == opponent for i in pattern if i != move):
                return True
        return False
    

    def ask_gemini_for_move(self):
        """Send current board to Gemini and get best move index."""
        board_map = "\n".join([
            f"{self.grid[0]} | {self.grid[3]} | {self.grid[6]}",
            f"{self.grid[1]} | {self.grid[4]} | {self.grid[7]}",
            f"{self.grid[2]} | {self.grid[5]} | {self.grid[8]}",
        ])
        
        prompt = f"""
You are a strong Tic Tac Toe AI (GREEN = 2). Human is BLUE = 1.
Grid numbering (column-major):
0 | 3 | 6
1 | 4 | 7
2 | 5 | 8

Current board (0=empty, 1=human, 2=you):
{board_map}

Your goals:
1. Win immediately if possible.
2. Block human (1) if they are about to win.
3. If neither, prioritize in this order:
   - Center (4)
   - Corners (0, 2, 6, 8)
   - Edges (1, 3, 5, 7)
4. Always choose an empty cell.
5. Respond with ONLY one integer (0–8), no text, no punctuation, no explanation.
"""

        try:
            response = self.model.generate_content(prompt)
            reply = response.text.strip()
            self.get_logger().info(f"🧩 Gemini suggested: {reply}")
            
            # Parse Gemini's response
            try:
                gemini_move = int(reply)
            except ValueError:
                self.get_logger().warn("⚠️ Gemini gave non-integer reply.")
                gemini_move = None

            heuristic_move = self.find_blocking_or_winning_move(self.grid, 2, 1)
            self.get_logger().info(f"🧠 Heuristic fallback: {heuristic_move}")

            # Compare and validate
            if gemini_move is not None and 0 <= gemini_move <= 8 and self.grid[gemini_move] == 0:
                if heuristic_move is not None and heuristic_move != gemini_move:
                    if self.is_better_move(self.grid, heuristic_move, gemini_move, 2, 1):
                        self.get_logger().info(f"⚔️ Overriding Gemini move {gemini_move} → Heuristic {heuristic_move}")
                        return heuristic_move
                return gemini_move

            # If Gemini move invalid → fallback
            self.get_logger().warn("⚠️ Invalid/weak Gemini move, using heuristic fallback.")
            if heuristic_move is not None:
                return heuristic_move

            # Last resort: pick first empty cell
            for i, val in enumerate(self.grid):
                if val == 0:
                    self.get_logger().info(f"🟩 Random fallback move: {i}")
                    return i

        except Exception as e:
            self.get_logger().error(f"❌ Gemini API Error: {e}")
            # Fallback after API failure
            fallback = self.find_blocking_or_winning_move(self.grid, 2, 1)
            if fallback is not None:
                return fallback
            for i, val in enumerate(self.grid):
                if val == 0:
                    return i
            return None


    def make_ai_move(self, move):
        """Publish AI move via /ai_move topic to mapper."""
        self.grid[move] = 2  # Update internal state
        move_msg = Int32()
        move_msg.data = move
        self.ai_move_pub.publish(move_msg)
        self.get_logger().info(f"🟩 AI chose and published move {move}")


# ==========================================================
# 🚀 ROS Entry Point
# ==========================================================
def main(args=None):
    rclpy.init(args=args)
    node = DecisionMaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
