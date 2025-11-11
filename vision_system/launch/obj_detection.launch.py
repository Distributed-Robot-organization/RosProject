from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def load_robot_names(context, *args, **kwargs):
    """Load robot names from shelfino_params.yaml"""
    config_file = LaunchConfiguration('config_file').perform(context)
    
    robot_names = ['shelfino1']  # Default
    
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
            if '/**' in config and 'ros__parameters' in config['/**']:
                params = config['/**']['ros__parameters']
                if 'init_names' in params:
                    robot_names = params['init_names']
    except Exception as e:
        print(f"Warning: Could not load config file, using default robot names. Error: {e}")
    
    # Create a detection node for each robot
    nodes = []
    for robot_name in robot_names:
        detection_node = Node(
            package='vision_system',
            executable='detection_pcl.py',
            name=f'detection_pcl_{robot_name}',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'robot_namespace': robot_name,
                'rgb_image_topic': f'/{robot_name}/f_camera/image_raw',
                'depth_image_topic': f'/{robot_name}/f_camera/depth/image_raw',
                'detection_image_topic': f'/{robot_name}/vision_system/yolo_detection_image',
                'detection_results_topic': f'/{robot_name}/vision_system/yolo_detection_results',
                'point_cloud_topic': f'/{robot_name}/f_camera/points',
                'info_camera': f'/{robot_name}/f_camera/camera_info',
                'z_ground_offset': 0.0,
            }]
        )
        nodes.append(detection_node)
    
    return nodes

def generate_launch_description():
    # Get the path to the shelfino_params.yaml
    dist_project_dir = get_package_share_directory('dist_project')
    default_config_file = os.path.join(dist_project_dir, 'config', 'shelfino_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Path to the shelfino_params.yaml configuration file'
        ),
        OpaqueFunction(function=load_robot_names),
    ])
    
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/shelfino1/cmd_vel

#   ros2 service call /shelfino1/trigger_detection vision_system/srv/NameObject "{name_object: 'small_cone'}"
#   ros2 service call /shelfino1/trigger_detection vision_system/srv/NameObject "{name_object: ''}" --> to deactivate

#   ros2 service call /shelfino1/trigger_pcl std_srvs/srv/Trigger "{}"
#   ros2 service call /pollo/trigger_pcl std_srvs/srv/Trigger "{}"

#   ros2 service call /shelfino1/trigger_filter_pcl std_srvs/srv/Trigger "{}"
#   ros2 service call /pollo/trigger_filter_pcl std_srvs/srv/Trigger "{}"

#   ros2 topic echo /shelfino1/vision_system/detected_objects
#   ros2 topic echo /pollo/vision_system/detected_objects