import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.time import Time
import math

class StartToGoalLineTimeMeasurer(Node):
    def __init__(self):
        super().__init__('start_to_goal_line_time_measurer')

        # --- パラメータの宣言 ---
        # スタートラインを定義する2点
        self.declare_parameter('start_line_p1_x', 0.0)
        self.declare_parameter('start_line_p1_y', 0.0)
        self.declare_parameter('start_line_p2_x', 0.0)
        self.declare_parameter('start_line_p2_y', 0.0)
        # フィニッシュラインを定義する2点
        self.declare_parameter('finish_line_p1_x', 0.0)
        self.declare_parameter('finish_line_p1_y', 0.0)
        self.declare_parameter('finish_line_p2_x', 0.0)
        self.declare_parameter('finish_line_p2_y', 0.0)

        # --- パラメータの取得 ---
        self.start_line = {
            'p1_x': self.get_parameter('start_line_p1_x').get_parameter_value().double_value,
            'p1_y': self.get_parameter('start_line_p1_y').get_parameter_value().double_value,
            'p2_x': self.get_parameter('start_line_p2_x').get_parameter_value().double_value,
            'p2_y': self.get_parameter('start_line_p2_y').get_parameter_value().double_value,
        }
        self.finish_line = {
            'p1_x': self.get_parameter('finish_line_p1_x').get_parameter_value().double_value,
            'p1_y': self.get_parameter('finish_line_p1_y').get_parameter_value().double_value,
            'p2_x': self.get_parameter('finish_line_p2_x').get_parameter_value().double_value,
            'p2_y': self.get_parameter('finish_line_p2_y').get_parameter_value().double_value,
        }

        self.get_logger().info(f"Measurement Start Line: ({self.start_line['p1_x']}, {self.start_line['p1_y']}) -> ({self.start_line['p2_x']}, {self.start_line['p2_y']})")
        self.get_logger().info(f"Measurement Finish Line: ({self.finish_line['p1_x']}, {self.finish_line['p1_y']}) -> ({self.finish_line['p2_x']}, {self.finish_line['p2_y']})")

        # --- Subscriberの作成 ---
        self.subscription = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odometry_callback,
            10)

        # --- 状態管理用の変数 ---
        self.state = "WAITING_FOR_START"  # WAITING_FOR_START -> TIMING -> FINISHED
        self.start_time = None
        self.previous_position = None

    def check_line_crossing(self, line, prev_pos, curr_pos):
        """車両の移動軌跡が指定されたラインと交差したかを判定する"""
        line_dx = line['p2_x'] - line['p1_x']
        line_dy = line['p2_y'] - line['p1_y']

        side_prev = line_dx * (prev_pos.y - line['p1_y']) - line_dy * (prev_pos.x - line['p1_x'])
        side_curr = line_dx * (curr_pos.y - line['p1_y']) - line_dy * (curr_pos.x - line['p1_x'])

        if side_prev * side_curr <= 0 and side_prev != 0 and side_curr != 0:
            vehicle_dx = curr_pos.x - prev_pos.x
            vehicle_dy = curr_pos.y - prev_pos.y
            side_p1 = vehicle_dx * (line['p1_y'] - prev_pos.y) - vehicle_dy * (line['p1_x'] - prev_pos.x)
            side_p2 = vehicle_dx * (line['p2_y'] - prev_pos.y) - vehicle_dy * (line['p2_x'] - prev_pos.x)
            if side_p1 * side_p2 <= 0:
                return True
        return False

    def odometry_callback(self, msg: Odometry):
        if self.state == "FINISHED":
            return

        current_pos = msg.pose.pose.position
        if self.previous_position is None:
            self.previous_position = current_pos
            return

        # --- ステートマシン ---
        if self.state == "WAITING_FOR_START":
            if self.check_line_crossing(self.start_line, self.previous_position, current_pos):
                self.start_time = self.get_clock().now()
                self.state = "TIMING"
                self.get_logger().info("Crossed the start line. Start measuring...")

        elif self.state == "TIMING":
            if self.check_line_crossing(self.finish_line, self.previous_position, current_pos):
                end_time = self.get_clock().now()
                elapsed_time = (end_time - self.start_time).nanoseconds / 1e9
                self.state = "FINISHED"

                self.get_logger().info("Crossed the finish line. Stop measuring.")
                self.get_logger().info(f"--- Elapsed Time: {elapsed_time:.3f} sec ---")
                
                # self.destroy_node()
                raise KeyboardInterrupt 

        self.previous_position = current_pos

def main(args=None):
    rclpy.init(args=args)
    time_measurer_node = StartToGoalLineTimeMeasurer()
    try:
        rclpy.spin(time_measurer_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            time_measurer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
