import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    param_file_name = "edtm_mapping.yaml"
    param_file = os.path.join(get_package_share_directory('plan_env'), 'param', param_file_name)
    print("default param file: ", param_file)
    param_dir = LaunchConfiguration('params_file', default=param_file)
    input_cloud = LaunchConfiguration('input_cloud', default="/front_camera/world/points")
    # input_cloud = LaunchConfiguration('input_cloud', default="/lidar/world/points")
    input_depth_img = LaunchConfiguration('input_depth_img', default="/front_camera/depth/image_raw")
    input_odom = LaunchConfiguration('input_odom', default="/odometry/global")
    
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=param_dir, description='Full path to edtm mapping param path'),
        DeclareLaunchArgument('input_cloud', default_value=input_cloud, description='Full path to edtm mapping param path'),
        DeclareLaunchArgument('input_odom', default_value=input_odom, description='Full path to edtm mapping param path'),
        DeclareLaunchArgument('input_depth_img', default_value=input_depth_img, description='Full path to edtm mapping param path'),
        # Node(
        #     package='plan_env',
        #     executable='edtf_mapping',
        #     name='edtf_mapping',
        #     parameters=[param_file],
        #     remappings=[("/grid_map/cloud", input_cloud), ("/grid_map/odom", input_odom),],
        #     # remappings=[("/grid_map/depth", input_depth_img), ("/grid_map/odom", input_odom),], # TODO need to add correct camera to body transformation 
        #     output='screen'),
        Node(
            package='pointcloud_processing',
            executable='point_cloud_transformer',
            name='point_cloud_transformer',
            parameters=[param_file], 
            output='screen')
    ])
