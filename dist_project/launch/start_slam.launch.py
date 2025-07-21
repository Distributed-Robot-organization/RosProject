import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
import yaml

def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    shelfino_name = ""
    params_path= os.path.join(get_package_share_directory('dist_project'), 'config', 'shelfino_params.yaml')
    with open (params_path, 'r') as f:
        shelfino_config_yaml = yaml.load(f, Loader=yaml.FullLoader)
        shelfino_config_yaml = shelfino_config_yaml["/**"]["ros__parameters"]
        shelfino_name = shelfino_config_yaml["init_names"][0]


    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            remappings=[
                ('/scan', '/'+shelfino_name+'/scan'),
                ('/odom', '/'+shelfino_name+'/odom')
            ],
            parameters=[
                {'use_sim_time': use_sim_time},  # or True, depending on your setup
                get_package_share_directory('dist_project')+'/config/slam.yaml'
            ]
        )
])
