import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.time import Time
import math
import yaml
from time_checker_msgs.srv import ResetTimer

class PolygonPassOppositeLineTimeMeasurer(Node):
    def __init__(self):
        super().__init__('polygon_pass_opposite_line_time_measurer')

        # --- パラメータの宣言と取得 ---
        self.declare_parameter('vehicle_length', 7.24)
        self.declare_parameter('vehicle_width', 2.30)
        self.declare_parameter('base_link_to_center_offset', 2.19)
        self.declare_parameter('config_file', '')
        self.declare_parameter('initial_location', '')

        self.vehicle_length = self.get_parameter('vehicle_length').get_parameter_value().double_value
        self.vehicle_width = self.get_parameter('vehicle_width').get_parameter_value().double_value
        self.base_link_to_center_offset = self.get_parameter('base_link_to_center_offset').get_parameter_value().double_value
        config_file_path = self.get_parameter('config_file').get_parameter_value().string_value
        initial_location = self.get_parameter('initial_location').get_parameter_value().string_value

        # --- YAMLファイルの読み込み ---
        self.all_locations_data = None
        try:
            with open(config_file_path, 'r') as f:
                all_params_yaml = yaml.safe_load(f)
                self.all_locations_data = all_params_yaml['polygon_pass_opposite_line_time_measurer']['ros__parameters']
            self.get_logger().info(f"Successfully loaded location data from {config_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load or parse YAML file {config_file_path}: {e}")
            self.state = "ERROR"
            return

        # --- 状態管理用変数 ---
        self.state = "IDLE"
        self.start_line = {}
        self.finish_line = {}
        self.start_time = None
        self.previous_corners = None
        # --- 変更点: ゴールラインに対する車両の初期サイドを保存する変数を追加 ---
        self.initial_finish_side_sign = None

        # --- Subscriber & Service Server ---
        self.subscription = self.create_subscription(
            Odometry, '/localization/kinematic_state', self.odometry_callback, 10)
        self.reset_service = self.create_service(ResetTimer, 'reset_timer', self.reset_service_callback)

        if initial_location:
            self._set_location(initial_location)
        else:
            self.get_logger().warn("No initial_location set. Call the 'reset_timer' service to start.")
        self.get_logger().info("Time measurer node is ready.")

    def _set_location(self, location_str):
        try:
            keys = location_str.split('.')
            location_params = self.all_locations_data
            for key in keys:
                location_params = location_params[key]

            self.start_line = {
                'p1': {'x': location_params['start_line_p1_x'], 'y': location_params['start_line_p1_y']},
                'p2': {'x': location_params['start_line_p2_x'], 'y': location_params['start_line_p2_y']},
            }
            self.finish_line = {
                'p1': {'x': location_params['finish_line_p1_x'], 'y': location_params['finish_line_p1_y']},
                'p2': {'x': location_params['finish_line_p2_x'], 'y': location_params['finish_line_p2_y']},
            }

            self.state = "WAITING_FOR_START"
            self.start_time = None
            self.previous_corners = None
            # --- 変更点: 状態変数をリセット ---
            self.initial_finish_side_sign = None

            self.get_logger().info(f"--- Location set to '{location_str}' ---")
            self.get_logger().info(f"Start Line (Segment): ({self.start_line['p1']['x']}, {self.start_line['p1']['y']}) -> ({self.start_line['p2']['x']}, {self.start_line['p2']['y']})")
            self.get_logger().info(f"Finish Line (State Check): ({self.finish_line['p1']['x']}, {self.finish_line['p1']['y']}) -> ({self.finish_line['p2']['x']}, {self.finish_line['p2']['y']})")
            return True, f"Successfully set location to {location_str}"
        except Exception as e:
            self.get_logger().error(f"Error setting location '{location_str}': {e}")
            self.state = "IDLE"
            return False, f"Error setting location: {e}"

    def reset_service_callback(self, request, response):
        success, message = self._set_location(request.location)
        response.success = success
        response.message = message
        return response

    def get_vehicle_corners(self, pose):
        x, y, q = pose.position.x, pose.position.y, pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        half_width = self.vehicle_width / 2.0
        front_offset = self.vehicle_length / 2.0 + self.base_link_to_center_offset
        rear_offset = self.vehicle_length / 2.0 - self.base_link_to_center_offset
        corners_local = [(front_offset, half_width), (front_offset, -half_width), (-rear_offset, -half_width), (-rear_offset, half_width)]
        return [{'x': x + lx * math.cos(yaw) - ly * math.sin(yaw), 'y': y + lx * math.sin(yaw) + ly * math.cos(yaw)} for lx, ly in corners_local]

    # --- 変更点: 点が直線のどちら側にあるかを判定する関数を再定義 ---
    def _get_point_side_of_line(self, point, line_p1, line_p2):
        """点が直線(line_p1 -> line_p2)のどちら側にあるかを判定"""
        return (line_p2['x'] - line_p1['x']) * (point['y'] - line_p1['y']) - \
               (line_p2['y'] - line_p1['y']) * (point['x'] - line_p1['x'])

    @staticmethod
    def on_segment(p, q, r):
        return (q['x'] <= max(p['x'], r['x']) and q['x'] >= min(p['x'], r['x']) and
                q['y'] <= max(p['y'], r['y']) and q['y'] >= min(p['y'], r['y']))

    @staticmethod
    def orientation(p, q, r):
        val = (q['y'] - p['y']) * (r['x'] - q['x']) - (q['x'] - p['x']) * (r['y'] - q['y'])
        if val == 0: return 0
        return 1 if val > 0 else 2

    def check_segment_intersection(self, p1, q1, p2, q2):
        o1, o2, o3, o4 = self.orientation(p1, q1, p2), self.orientation(p1, q1, q2), self.orientation(p2, q2, p1), self.orientation(p2, q2, q1)
        if o1 != o2 and o3 != o4: return True
        if o1 == 0 and self.on_segment(p1, p2, q1): return True
        if o2 == 0 and self.on_segment(p1, q2, q1): return True
        if o3 == 0 and self.on_segment(p2, p1, q2): return True
        if o4 == 0 and self.on_segment(p2, q1, q2): return True
        return False

    def odometry_callback(self, msg: Odometry):
        if self.state not in ["WAITING_FOR_START", "TIMING"]:
            return

        current_corners = self.get_vehicle_corners(msg.pose.pose)
        if self.previous_corners is None:
            self.previous_corners = current_corners
            return

        if self.state == "WAITING_FOR_START":
            is_crossing_start = False
            for i in range(4):
                if self.check_segment_intersection(self.previous_corners[i], current_corners[i], self.start_line['p1'], self.start_line['p2']):
                    is_crossing_start = True
                    break
            
            if is_crossing_start:
                self.start_time = self.get_clock().now()
                self.state = "TIMING"
                self.get_logger().info("Vehicle crossed start line segment. Timer started.")

                # --- 変更点: 計測開始時の、ゴールラインに対するサイドを記録 ---
                sides = [self._get_point_side_of_line(c, self.finish_line['p1'], self.finish_line['p2']) for c in current_corners]
                significant_sides = [s for s in sides if abs(s) > 1e-6] # 誤差を考慮
                if significant_sides:
                    self.initial_finish_side_sign = math.copysign(1.0, significant_sides[0])
                    self.get_logger().info(f"Determined initial side relative to finish line: {self.initial_finish_side_sign}")
                else:
                    self.get_logger().warn("Vehicle started exactly on the finish line. Measurement might be inaccurate.")
                    # 便宜的に符号を1とする
                    self.initial_finish_side_sign = 1.0

        elif self.state == "TIMING":
            if self.initial_finish_side_sign is None:
                self.get_logger().error("Cannot determine finish condition, initial side was not set. Aborting.", throttle_duration_sec=5)
                self.state = "IDLE"
                return

            # --- 変更点: 全ての角が、初期サイドの「逆側」に移動したかチェック ---
            target_sign = -self.initial_finish_side_sign
            sides = [self._get_point_side_of_line(c, self.finish_line['p1'], self.finish_line['p2']) for c in current_corners]
            significant_sides = [s for s in sides if abs(s) > 1e-6]
            
            # 全ての有効な点が目標のサイドにあるか
            if significant_sides and all(math.copysign(1.0, s) == target_sign for s in significant_sides):
                end_time = self.get_clock().now()
                elapsed_time = (end_time - self.start_time).nanoseconds / 1e9
                self.state = "FINISHED"

                self.get_logger().info("Vehicle has completely crossed the finish line. Timer stopped.")
                self.get_logger().info(f"\033[1;92m--- Elapsed Time: {elapsed_time:.3f} sec ---\033[0m")
                self.get_logger().info("To measure again, call the 'reset_timer' service.")

        self.previous_corners = current_corners

# main関数は変更なし
def main(args=None):
    rclpy.init(args=args)
    time_measurer_node = PolygonPassOppositeLineTimeMeasurer()
    if time_measurer_node.state != "ERROR":
        try:
            rclpy.spin(time_measurer_node)
        except KeyboardInterrupt:
            pass
        finally:
            if rclpy.ok():
                time_measurer_node.destroy_node()
            rclpy.shutdown()
    else:
        time_measurer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()