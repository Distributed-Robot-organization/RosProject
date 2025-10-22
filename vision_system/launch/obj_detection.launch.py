from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    detection = Node(
        package='vision_system',
        executable='detection_pcl.py',  
        name='detection_pcl',
        output='screen',
        parameters=[{
            'rgb_image_topic': '/shelfino1/f_camera/image_raw',
            'depth_image_topic': '/shelfino1/f_camera/depth/image_raw',
            'detection_image_topic': 'vision_system/yolo_detection_image',
            'detection_results_topic': 'vision_system/yolo_detection_results',
            
        }]
    )
    return LaunchDescription([
        detection,
    ])
    
    #ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/shelfino1/cmd_vel
    
    #ros2 service call /trigger_pcl std_srvs/srv/Trigger

    #ros2 service call /trigger_detection std_srvs/srv/Trigger
    