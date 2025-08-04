import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.time import Time
import math

class GoalLineTimeMeasurer(Node):
    def __init__(self):
        super().__init__('goal_line_time_measurer')

        # --- パラメータの宣言 (ラインを定義する2点) ---
        self.declare_parameter('line_p1_x', 0.0)
        self.declare_parameter('line_p1_y', 0.0)
        self.declare_parameter('line_p2_x', 0.0)
        self.declare_parameter('line_p2_y', 0.0)

        # --- パラメータの取得 ---
        self.p1_x = self.get_parameter('line_p1_x').get_parameter_value().double_value
        self.p1_y = self.get_parameter('line_p1_y').get_parameter_value().double_value
        self.p2_x = self.get_parameter('line_p2_x').get_parameter_value().double_value
        self.p2_y = self.get_parameter('line_p2_y').get_parameter_value().double_value

        self.get_logger().info(
            f"判定ライン: ({self.p1_x}, {self.p1_y}) から ({self.p2_x}, {self.p2_y}) まで"
        )

        # --- Subscriberの作成 ---
        self.subscription = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odometry_callback,
            10)

        # --- 状態管理用の変数 ---
        self.start_time = None
        self.is_timing = False
        self.is_finished = False
        self.previous_position = None # 1ステップ前の位置を保存する変数

    def check_line_crossing(self, prev_pos, curr_pos):
        """
        車両の移動軌跡(prev_pos -> curr_pos)が、
        定義したライン(p1 -> p2)と交差したかを判定する。
        """
        # ラインP1->P2のベクトル
        line_dx = self.p2_x - self.p1_x
        line_dy = self.p2_y - self.p1_y

        # P1から見た各点の位置ベクトルとの外積を計算し、ラインのどちら側にあるかを判定
        # 値の符号によって、点がラインのどちら側にあるかがわかる
        side_prev = line_dx * (prev_pos.y - self.p1_y) - line_dy * (prev_pos.x - self.p1_x)
        side_curr = line_dx * (curr_pos.y - self.p1_y) - line_dy * (curr_pos.x - self.p1_x)

        # 2つの点の符号が異なる場合、ラインをまたいだと判定
        if side_prev * side_curr <= 0:
            # さらに、車両の移動パスがラインと交差しているかを確認（より厳密なチェック）
            # 車両の移動パスのベクトル
            vehicle_dx = curr_pos.x - prev_pos.x
            vehicle_dy = curr_pos.y - prev_pos.y
            # prev_posから見た各点の位置ベクトルとの外積を計算
            side_p1 = vehicle_dx * (self.p1_y - prev_pos.y) - vehicle_dy * (self.p1_x - prev_pos.x)
            side_p2 = vehicle_dx * (self.p2_y - prev_pos.y) - vehicle_dy * (self.p2_x - prev_pos.x)
            if side_p1 * side_p2 <= 0:
                return True # 交差している

        return False

    def odometry_callback(self, msg: Odometry):
        if self.is_finished:
            return

        current_pos = msg.pose.pose.position
        current_speed = msg.twist.twist.linear.x

        if not self.is_timing and current_speed > 0.1:
            self.start_time = self.get_clock().now()
            self.is_timing = True
            self.get_logger().info("Vehicle starts to move. Start measuring...")

        # 最初のコールバックではprevious_positionがNoneなので、現在地を記録して終了
        if self.previous_position is None:
            self.previous_position = current_pos
            return
            
        if self.is_timing:
            # ラインを通過したかチェック
            if self.check_line_crossing(self.previous_position, current_pos):
                end_time = self.get_clock().now()
                elapsed_time = (end_time - self.start_time).nanoseconds / 1e9

                self.get_logger().info(f"Crossed the chosen line. Stop measuring.")
                self.get_logger().info(f"--- Elapsed Time: {elapsed_time:.2f} sec ---")

                self.is_finished = True
                # self.destroy_node()
                raise KeyboardInterrupt 
        
        # 次のステップのために現在位置を保存
        self.previous_position = current_pos


def main(args=None):
    rclpy.init(args=args)
    time_measurer_node = GoalLineTimeMeasurer()
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
