from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    #launch node build_dataset.py
    build_test = Node(
        package='dr_vision',
        executable='build_dataset.py',
        name='image_capture_node',
        output='screen'
    )
    yolo_pcl_node = Node(
        package='dr_vision',
        executable='yolo_pcl.py',  
        name='yolo_pcl_node',
        output='screen',
        parameters=[{
            'rgb_image_topic': '/camera/image_raw',
            'point_cloud_topic': '/camera/points',
            'info_camera': '/camera/camera_info',
            'detection_image_topic': 'dr_vision/yolo_detection_image',
            'detection_results_topic': 'dr_vision/yolo_detection_results'
        }]
    )
    return LaunchDescription([
        # build_test, #not possible to save in right folder using ros2
        yolo_pcl_node,
    ])
    
    #ros2 run teleop_twist_keyboard teleop_twist_keyboard