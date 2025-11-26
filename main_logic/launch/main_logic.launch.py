import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    
    main_logic_project_pkg = get_package_share_directory('main_logic')
    obj_params = os.path.join(main_logic_project_pkg, 'config', 'object_params.yaml')
    
    dist_project_pkg = get_package_share_directory('dist_project')
    obj_params_shelfino = os.path.join(dist_project_pkg, 'config', 'shelfino_params.yaml')
    
    return LaunchDescription([
        Node(
            package='main_logic',
            executable='main_logic_node',
            name='main_logic_node',
            output='screen',
            parameters=[{'yaml_path': obj_params,
                         'shelfino_params_path': obj_params_shelfino}]
        )
    ])

# ros2 topic pub /main_logic/obj_to_detect std_msgs/msg/String "data: 'small_cone'" --once