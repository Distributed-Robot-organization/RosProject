import launch
import launch_ros
from launch.actions import TimerAction

def generate_launch_description():
    navigation_system_node = TimerAction(
        period=0.01,
        actions=[
            launch_ros.actions.Node(
                package='navigation_system',
                executable='navigation_system_node',
                name='navigation_system_node',
                output='screen'
            )
        ]
    )

    nodes = [
        navigation_system_node,
    ]

    return launch.LaunchDescription(nodes)