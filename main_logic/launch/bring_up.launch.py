

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def include(pkg, launch_file, delay=0.0, **kwargs):
    
    action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), 'launch', launch_file])
        ),
        launch_arguments=kwargs.items()
    )
    return GroupAction([TimerAction(period=delay, actions=[action])]) if delay else action


def generate_launch_description():
    return LaunchDescription([
        # 2) Visione
        include('vision_system', 'obj_detection.launch.py', delay=1.0),

        # 3) Navigation
        include('navigation_system', 'navigation_system.launch.py', delay=5.0),

        # 4) Coordiantion
        include('coordination', 'coordinator.launch.py', delay=10.0),
        
        # 5) Main Logic
        include('main_logic', 'main_logic.launch.py', delay=15.0),
        
    ])

#   ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/shelfino1/cmd_vel    
# colcon build --symlink-install --parallel-workers 1 && source install/setup.bash 
# ros2 launch main_logic bring_up.launch.py 
# ros2 topic pub /main_logic/obj_to_detect std_msgs/msg/String "data: 'cone'" --once
# ros2 topic pub /main_logic/obj_to_detect std_msgs/msg/String "data: 'hydrant'" --once
# ros2 topic pub /main_logic/obj_to_detect std_msgs/msg/String "data: 'dumpster'" --once
# ros2 topic pub /main_logic/obj_to_detect std_msgs/msg/String "data: 'fountain'" --once