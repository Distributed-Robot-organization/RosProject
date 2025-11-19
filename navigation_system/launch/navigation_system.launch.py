import launch
import launch_ros
from launch.actions import TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml
import os
from ament_index_python.packages import get_package_share_directory

def load_robot_names(context):
    """Load robot names from the YAML configuration file."""
    config_file = LaunchConfiguration('config_file').perform(context)
    
    if not os.path.exists(config_file):
        print(f"Warning !!!! NO NODI PER PIù SHELFINI: Config file not found at {config_file}, using defaults")
        return ['shelfino1']
    
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # Extract robot names from the configuration
    if '/**' in config and 'ros__parameters' in config['/**']:
        params = config['/**']['ros__parameters']
        if 'init_names' in params:
            return params['init_names']
    
    return ['shelfino1']

def generate_navigation_nodes(context):
    """Generate navigation system nodes for each robot."""
    robot_names = load_robot_names(context)
    nodes = []
    
    for robot_name in robot_names:
        node = launch_ros.actions.Node(
            package='navigation_system',
            executable='navigation_system_node',
            name=f'navigation_system_{robot_name}',
            output='screen',
            parameters=[{'robot_namespace': robot_name}]
        )
        
        # Wrap each node in a TimerAction for delayed start
        timed_node = TimerAction(
            period=0.01,
            actions=[node]
        )
        nodes.append(timed_node)
    
    return nodes

def generate_launch_description():
    # Get the default config file path
    dist_project_share = get_package_share_directory('dist_project')
    default_config_file = os.path.join(dist_project_share, 'config', 'shelfino_params.yaml')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the robot configuration YAML file'
    )
    
    # Use OpaqueFunction to dynamically generate nodes based on config
    generate_nodes = OpaqueFunction(function=generate_navigation_nodes)
    
    return launch.LaunchDescription([
        config_file_arg,
        generate_nodes
    ])
    
#  new command
#  ros2 service call /shelfino1/rotate_shelfino navigation_system/srv/CenterPoint "{center: {x: 0.0, y: 0.0, z: 0.0}}"
#  ros2 service call /shelfino1/stop_navigation std_srvs/srv/Trigger "{}"
#  ros2 service call /shelfino1/pause_navigation std_srvs/srv/Trigger "{}"
#  ros2 service call /shelfino1/rotate_shelfino std_srvs/srv/Trigger "{}"
#  ros2 service call /shelfino1/generate_random_path std_srvs/srv/Trigger "{}"
#  ros2 service call /shelfino1/generate_specific_path navigation_system/srv/NavigateToGoal '{pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
#  ros2 service call /shelfino1/generate_arc navigation_system/srv/NavigateArc "{goal: {x: 4.0, y: 4.0, z: 0.0}, center: {x: 0.0, y: 0.0, z: 0.0}, radius: 4.0}"
