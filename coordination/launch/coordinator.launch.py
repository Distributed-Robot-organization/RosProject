import launch
import launch_ros
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    coordinator_node = launch_ros.actions.Node(
        package='coordination',
        executable='coordinator.py', 
        name='coordinator_node',
        output='screen',
    )

    nodes = [coordinator_node]
    
    return launch.LaunchDescription(nodes)


# ros2 service call /trigger_coordination_next_pose std_srvs/srv/Trigger "{}"
# ros2 service call /visualize_raw_ply_files std_srvs/srv/Trigger "{}"