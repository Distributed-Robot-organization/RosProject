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
    dist_project_pkg        = get_package_share_directory('dist_project')
    configs = {}


    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    configs["map_config"] = os.path.join(dist_project_pkg, 'config', 'single_shelfino.yaml')
    nodes_to_launch = []
    with open (configs["map_config"], 'r') as f:
        shelfino_config_yaml = yaml.load(f, Loader=yaml.FullLoader)
        shelfino_config_yaml = shelfino_config_yaml["/**"]["ros__parameters"]
        print(shelfino_config_yaml)
        for shelfino in range(len(shelfino_config_yaml['init_names'])):
            shelfino_name = shelfino_config_yaml['init_names'][shelfino]
            
            nodes_to_launch.append(Node(
                package='dist_project',
                executable='pcl_filter_node',
                name='pcl_filter_node',
                output='screen',
                namespace = shelfino_name,
                parameters=[
                    {'use_sim_time': use_sim_time},  # or True, depending on your setup
                    {"cloud_topic":"/"+shelfino_name+"/f_camera/points"},
                    {"world_frame":"map"}
                ]
            ))
        

    return LaunchDescription(
        nodes_to_launch
)
