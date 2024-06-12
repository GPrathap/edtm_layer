import os
import yaml 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_NAME = os.environ['ROBOT_NAME']

def generate_launch_description():

    param_file_name = ROBOT_NAME + '.yaml'
    param_file = os.path.join(get_package_share_directory('edtm_layer'), 'param', param_file_name)
    print("=========================param file", param_file)
    param_dir = LaunchConfiguration('params_file', default=param_file)

   
    with open(param_file, 'r') as f:
        params = yaml.safe_load(f)['elevation_mapping']['ros__parameters']
    
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=param_file, description='Full path to mapping param path'),
        Node(
            package='edtm_layer',
            executable='elevation_mapping',
            name='elevation_mapping',
            parameters=[params],
            output='screen'),
    ])
