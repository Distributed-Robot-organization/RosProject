from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os, yaml

def load_robot_names(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    
    robot_names = ['shelfino1']  # Default
    
    try: 
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
            params = config.get('/**', {}).get('ros__parameters', {})
            robot_names = params.get('init_names', robot_names) 
    except Exception as e:
        print(f"Error loading robot names: {e}")

    nodes = []

    coordinator_node = Node(
        package='coordination',
        executable='coordinator.py',
        name='coordinator_node',
        output='screen',
        parameters=[{
            'robot_namespaces': robot_names
        }]
    )

    nodes.append(coordinator_node)
    return nodes


def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('dist_project'),
        'config',
        'shelfino_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_path,
            description='Robot names config file'
        ),

        OpaqueFunction(function=load_robot_names),
    ])


# ros2 service call /trigger_coordination_next_pose std_srvs/srv/Trigger "{}"
# ros2 service call /visualize_raw_ply_files std_srvs/srv/Trigger "{}"