import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('time_checker')
    param_file_path = os.path.join(pkg_share, 'config', 'turn_spot.yaml')
    location_str = context.launch_configurations['location']

    # YAMLから車両の基本パラメータのみを読み込む
    with open(param_file_path, 'r') as f:
        all_params_yaml = yaml.safe_load(f)
    node_params_yaml = all_params_yaml['polygon_pass_opposite_line_time_measurer']['ros__parameters']

    # ノードに渡すパラメータを構築
    final_params = {
        'vehicle_length': node_params_yaml.get('vehicle_length'),
        'vehicle_width': node_params_yaml.get('vehicle_width'),
        'base_link_to_center_offset': node_params_yaml.get('base_link_to_center_offset'),
        'config_file': param_file_path,  # YAMLファイルのフルパスを渡す
        'initial_location': location_str, # 起動時のlocationを渡す
    }

    time_measurer_node = Node(
        package='time_checker',
        executable='polygon_pass_opposite_line_time_measurer',
        name='polygon_pass_opposite_line_time_measurer', # サービスコールしやすいようにノード名を固定
        parameters=[final_params],
        output='screen'
    )
    
    return [time_measurer_node]

def generate_launch_description():
    location_arg = DeclareLaunchArgument(
        'location',
        default_value='shiojiri.city_hall_entrance',
        description='Initial location key to use from the YAML file'
    )

    return LaunchDescription([
        location_arg,
        OpaqueFunction(function=launch_setup)
    ])