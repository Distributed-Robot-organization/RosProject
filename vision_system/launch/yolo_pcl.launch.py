from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    yolo_pcl = Node(
        package='vision_system',
        executable='yolo_pcl.py',  
        name='yolo_pcl',
        output='screen',
        parameters=[{
            'rgb_image_topic': '/shelfino1/f_camera/image_raw',
            'point_cloud_topic': '/shelfino1/f_camera/points',
            'info_camera': '/shelfino1/f_camera/camera_info',
            'detection_image_topic': 'vision_system/yolo_detection_image',
            'detection_results_topic': 'vision_system/yolo_detection_results'
        }]
    )
    return LaunchDescription([
        yolo_pcl,
    ])
    
    #ros2 run teleop_twist_keyboard teleop_twist_keyboard