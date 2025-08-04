import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.time import Time
import math
import yaml
from time_checker_msgs.srv import ResetTimer # 作成したサービスをインポート

class PolygonPassOppositeLineTimeMeasurer(Node):
    def __init__(self):
        super().__init__('polygon_pass_opposite_line_time_measurer')

        # --- パラメータの宣言と取得 ---
        self.declare_parameter('vehicle_length', 7.24)
        self.declare_parameter('vehicle_width', 2.30)
        self.declare_parameter('base_link_to_center_offset', 2.19)
        self.declare_parameter('config_file', '') # YAMLファイルのフルパス
        self.declare_parameter('initial_location', '') # 起動時のlocation

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
            self.state = "ERROR" # エラー状態を設定
            return

        # --- 状態管理用変数 ---
        self.state = "IDLE"  # IDLE -> WAITING_FOR_START -> TIMING -> FINISHED
        self.start_line = {}
        self.finish_line = {}
        self.start_time = None
        self.initial_side_sign = None
        self.intermediate_side_sign_for_finish_line = None

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            Odometry, '/localization/kinematic_state', self.odometry_callback, 10)

        # --- Service Server ---
        self.reset_service = self.create_service(ResetTimer, 'reset_timer', self.reset_service_callback)

        # --- 初期位置の設定 ---
        if initial_location:
            self.get_logger().info(f"Setting initial location to '{initial_location}'...")
            self._set_location(initial_location)
        else:
            self.get_logger().warn("No initial_location set. Call the 'reset_timer' service to start.")

        self.get_logger().info("Time measurer node is ready.")
        self.get_logger().info("To (re)start a measurement, call the 'reset_timer' service.")

    def _set_location(self, location_str):
        """指定されたlocationの座標を設定し、計測状態をリセットする内部関数"""
        try:
            keys = location_str.split('.')
            location_params = self.all_locations_data
            for key in keys:
                location_params = location_params[key]

            self.start_line = {
                'p1_x': location_params['start_line_p1_x'], 'p1_y': location_params['start_line_p1_y'],
                'p2_x': location_params['start_line_p2_x'], 'p2_y': location_params['start_line_p2_y'],
            }
            self.finish_line = {
                'p1_x': location_params['finish_line_p1_x'], 'p1_y': location_params['finish_line_p1_y'],
                'p2_x': location_params['finish_line_p2_x'], 'p2_y': location_params['finish_line_p2_y'],
            }

            # 状態をリセット
            self.state = "WAITING_FOR_START"
            self.start_time = None
            self.initial_side_sign = None
            self.intermediate_side_sign_for_finish_line = None

            self.get_logger().info(f"--- Location set to '{location_str}' ---")
            self.get_logger().info(f"Measurement Start Line: ({self.start_line['p1_x']}, {self.start_line['p1_y']}) -> ({self.start_line['p2_x']}, {self.start_line['p2_y']})")
            self.get_logger().info(f"Measurement Finish Line: ({self.finish_line['p1_x']}, {self.finish_line['p1_y']}) -> ({self.finish_line['p2_x']}, {self.finish_line['p2_y']})")
            self.get_logger().info("State is now WAITING_FOR_START. Move the vehicle completely behind the start line.")
            return True, f"Successfully set location to {location_str}"
        except KeyError:
            self.get_logger().error(f"Location '{location_str}' not found in the config file.")
            self.state = "IDLE"
            return False, f"Location '{location_str}' not found."
        except Exception as e:
            self.get_logger().error(f"An error occurred while setting location: {e}")
            self.state = "IDLE"
            return False, f"An unexpected error occurred: {e}"

    def reset_service_callback(self, request, response):
        """サービスコールに応じて場所を設定しタイマーをリセットする"""
        self.get_logger().info(f"Reset service called with location: '{request.location}'")
        success, message = self._set_location(request.location)
        response.success = success
        response.message = message
        return response

    def get_vehicle_corners(self, pose):
        """車両の中心位置と向きから四隅の座標を計算する"""
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        half_width = self.vehicle_width / 2.0
        front_offset = self.vehicle_length / 2.0 + self.base_link_to_center_offset
        rear_offset = self.vehicle_length / 2.0 - self.base_link_to_center_offset
        corners_local = [
            (front_offset, half_width), (front_offset, -half_width),
            (-rear_offset, -half_width), (-rear_offset, half_width)
        ]
        corners_global = []
        for lx, ly in corners_local:
            gx = x + lx * math.cos(yaw) - ly * math.sin(yaw)
            gy = y + lx * math.sin(yaw) + ly * math.cos(yaw)
            corners_global.append({'x': gx, 'y': gy})
        return corners_global

    def get_point_side_of_line(self, point, line):
        """点がラインのどちら側にあるかを判定する (外積を利用)"""
        return (line['p2_x'] - line['p1_x']) * (point['y'] - line['p1_y']) - \
               (line['p2_y'] - line['p1_y']) * (point['x'] - line['p1_x'])

    def odometry_callback(self, msg: Odometry):
        if self.state not in ["WAITING_FOR_START", "TIMING"]:
            return

        vehicle_corners = self.get_vehicle_corners(msg.pose.pose)

        if self.state == "WAITING_FOR_START":
            sides = [self.get_point_side_of_line(corner, self.start_line) for corner in vehicle_corners]
            if self.initial_side_sign is None:
                significant_sides = [s for s in sides if s != 0]
                if not significant_sides: return
                if all(math.copysign(1.0, s) == math.copysign(1.0, significant_sides[0]) for s in significant_sides):
                    self.initial_side_sign = math.copysign(1.0, significant_sides[0])
                    self.get_logger().info(f"Vehicle is positioned correctly. Ready to measure. (Initial side sign: {self.initial_side_sign})")
                else:
                    self.get_logger().warn("Vehicle started on the line. Please move completely to one side.", throttle_duration_sec=5)
                return

            is_crossing = any(math.copysign(1.0, s) != self.initial_side_sign for s in sides if s != 0)
            if is_crossing:
                self.start_time = self.get_clock().now()
                self.state = "TIMING"
                self.get_logger().info("Vehicle crossed the start line. Start measuring...")
                finish_line_sides = [self.get_point_side_of_line(corner, self.finish_line) for corner in vehicle_corners]
                first_significant_finish_side = next((s for s in finish_line_sides if s != 0), None)
                if first_significant_finish_side is not None:
                    self.intermediate_side_sign_for_finish_line = math.copysign(1.0, first_significant_finish_side)
                    self.get_logger().info(f"Entered intermediate area. Finish line sign relative to vehicle: {self.intermediate_side_sign_for_finish_line}")

        elif self.state == "TIMING":
            if self.intermediate_side_sign_for_finish_line is None:
                self.get_logger().error("Could not determine intermediate side for finish line. Aborting.", throttle_duration_sec=5)
                self.state = "IDLE"
                return
            
            sides = [self.get_point_side_of_line(corner, self.finish_line) for corner in vehicle_corners]
            finish_side_sign = -self.intermediate_side_sign_for_finish_line
            significant_sides = [s for s in sides if s != 0]
            if significant_sides and all(math.copysign(1.0, s) == finish_side_sign for s in significant_sides):
                end_time = self.get_clock().now()
                elapsed_time = (end_time - self.start_time).nanoseconds / 1e9
                self.state = "FINISHED" # 状態を FINISHED に変更

                self.get_logger().info("Vehicle completely crossed the finish line. Stop measuring.")
                self.get_logger().info(f"\033[1;92m--- Elapsed Time: {elapsed_time:.3f} sec ---\033[0m")
                self.get_logger().info("To measure again, call the 'reset_timer' service.")
                # raise KeyboardInterrupt を削除

def main(args=None):
    rclpy.init(args=args)
    time_measurer_node = PolygonPassOppositeLineTimeMeasurer()
    # YAML読み込みなどでエラーが発生した場合はspinせずに終了
    if time_measurer_node.state != "ERROR":
        try:
            rclpy.spin(time_measurer_node)
        except KeyboardInterrupt:
            pass # Ctrl+Cで正常終了
        finally:
            if rclpy.ok():
                time_measurer_node.destroy_node()
            rclpy.shutdown()
    else:
        time_measurer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()