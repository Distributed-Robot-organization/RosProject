#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import geometry_msgs.msg

import numpy as np
import struct
from cv_bridge import CvBridge
import cv2
import tf2_ros
import tf2_geometry_msgs
import datetime

from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2 as pc2
from builtin_interfaces.msg import Time as BuiltinTime


from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from vision_system.msg import ObjectDetectionBox, ObjectDetectionResult

import sys
from utility_ply import PointCloudManager


class ObjectDetectionNode(Node):

    def __init__(self) -> None:
        super().__init__('object_detection_node')
        
        # Set PLY output directory to /ros2_ws
        self.ply_out_dir = '/ros2_ws/src/vision_system/script'
        os.makedirs(self.ply_out_dir, exist_ok=True)

        # import the model
        package_share_directory = get_package_share_directory('vision_system')
        model_path = os.path.join(package_share_directory, 'models', 'best.pt')
        self.model = YOLO(model_path)
        
        #for each shelfino
        self.declare_parameter('rgb_image_topic', '/shelfino1/f_camera/image_raw')
        self.declare_parameter('depth_image_topic', '/shelfino1/f_camera/depth/image_raw')
        
        self.declare_parameter('detection_image_topic', 'vision_system/yolo_detection_image')
        self.declare_parameter('detection_results_topic', 'vision_system/yolo_detection_results')
        
        self.declare_parameter('point_cloud_topic', '/shelfino1/f_camera/points')
        self.declare_parameter('info_camera', '/shelfino1/f_camera/camera_info')

        # Get parameter values
        rgb_topic = self.get_parameter('rgb_image_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_image_topic').get_parameter_value().string_value
        image_detection_topic = self.get_parameter('detection_image_topic').get_parameter_value().string_value
        detection_results_topic = self.get_parameter('detection_results_topic').get_parameter_value().string_value
        point_cloud_topic = self.get_parameter('point_cloud_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('info_camera').get_parameter_value().string_value

        # QoS settings for the subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=20
        )
        
        # == SUBSCRIBERS ==
        self.rgb_subscription = self.create_subscription(
            Image,
            rgb_topic,
            self.rgb_callback,
            qos_profile
        )
        self.depth_subscription = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            qos_profile
        )    
        self.pc_subscription = self.create_subscription(
            PointCloud2, 
            point_cloud_topic, 
            self.pointcloud_callback, 
            qos_profile
        )      
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            camera_info_topic, 
            self.camera_info_callback, 
            qos_profile
        )
        
        # == PUBLISHER ==
        self.image_detetection_pub = self.create_publisher(Image, image_detection_topic, 1)
        self.detection_res_pub = self.create_publisher(ObjectDetectionResult,detection_results_topic, 1)
        
        
        # == TRIGGER SERVICES ==
        self.srv_detection = self.create_service(Trigger, 'trigger_detection', self.trigger_detection_callback)
        self.srv_pcl = self.create_service(Trigger, 'trigger_pcl', self.trigger_pcl_callback)
        self.srv_filter_pcl = self.create_service(Trigger, 'trigger_filter_pcl', self.trigger_filter_pcl_callback)
        
        self.detection_triggered = False
        self.pcl_triggered = False
        
        # == Parameters ==
        self.confidence_threshold = 0.5
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.bridge = CvBridge()
        self.depth_image = None
        
        self.last_rgb_msg = None
        self.last_cv_image = None
        self.last_cloud = None
        self.last_cam_info = None
        
        self.detected_objects_list = []
        self.pcl_manager = PointCloudManager()
        self.get_logger().info("Object Detection Node with Depth and map Transform has been started.")
        
    # == detection part ==
    def trigger_detection_callback(self, request, response):
        self.get_logger().info('Trigger received!')
        self.detection_triggered = not self.detection_triggered
        if self.detection_triggered:
            self.get_logger().info('Object detection activated.')
            response.success = True
            response.message = 'Object detection activated successfully.'
        else:
            self.get_logger().info('Object detection deactivated.')
            response.success = False
            response.message = 'Object detection deactivated.'
        return response

    def camera_info_callback(self, info: CameraInfo) -> None:
        self.last_cam_info = info

    def depth_callback(self, depth_data: Image) -> None:
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")

    def rgb_callback(self, rgb_data: Image) -> None:
        # Check if detection is triggered
        if not self.detection_triggered:
            return
        
        self.last_rgb_msg = rgb_data
        self.detected_objects_list.clear()
        
        cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        self.last_cv_image = cv_image
        
        results = self.model(cv_image)

        if len(results) > 0 and results[0].boxes is not None:
            
            boxes = results[0].boxes
            annotated_frame = cv_image.copy()

            detection_msg = ObjectDetectionResult()
            detection_msg.header = Header()
            detection_msg.header.stamp = self.get_clock().now().to_msg()
            detection_msg.header.frame_id = "object_detection"

            for i, box in enumerate(boxes):
                xyxy = box.xyxy[0]
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                
                if conf < self.confidence_threshold:
                    continue
                
                label = results[0].names[cls_id] if hasattr(results[0], 'names') else str(cls_id)

                x_min, y_min, x_max, y_max = xyxy
                cx = int((x_min + x_max) / 2.0)
                cy = int((y_min + y_max) / 2.0)

                color = (0, 255, 0)  # Verde
                cv2.rectangle(annotated_frame, 
                            (int(x_min), int(y_min)), 
                            (int(x_max), int(y_max)), 
                            color, 2)
                label_text = f"{label} {conf:.2f}"
                
                cv2.putText(annotated_frame, label_text, 
                          (int(x_min), int(y_min) - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                x_3d, y_3d, z_3d, map_x, map_y, map_z, distance = self.yolo_detection(
                    cx, cy, x_min, y_max, annotated_frame, rgb_data
                )
                cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 0), -1)
                
                # Messaggio di detection
                box_msg = ObjectDetectionBox()
                box_msg.id = i
                box_msg.label = label
                box_msg.confidence = conf
                box_msg.x_min = float(x_min)
                box_msg.y_min = float(y_min)
                box_msg.x_max = float(x_max)
                box_msg.y_max = float(y_max)
                box_msg.distance = distance
                box_msg.world_x = map_x
                box_msg.world_y = map_y
                box_msg.world_z = map_z

                detection_msg.boxes.append(box_msg)
                
                # Extract 3D points from bounding box
                points_3d = self.extract_bbox_pointcloud(int(x_min), int(y_min), int(x_max), int(y_max))
                
                detected_objects = {
                    'id': i,
                    'label': label,
                    'confidence': conf,
                    'bbox': [float(x_min), float(y_min), float(x_max), float(y_max)],
                    'distance': distance,
                    'world_coordinates': {
                        'x': map_x,
                        'y': map_y,
                        'z': map_z
                    },
                    'pcl_object': points_3d
                }
                self.detected_objects_list.append(detected_objects)
                self.publish_tf(map_x, map_y, map_z, f"{label}_{i}")

            # Pubblica SOLO se ci sono oggetti sopra la soglia
            if len(detection_msg.boxes) > 0:
                self.detection_res_pub.publish(detection_msg)
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
                self.image_detetection_pub.publish(annotated_msg)
            else:
                self.get_logger().info("No objects detected above confidence threshold.")
        else:
            self.get_logger().info("I don't see any objects.")

    def yolo_detection(self, cx, cy, x_min, y_max, annotated_frame, rgb_data):
        x_3d = y_3d = z_3d = None
        map_x = map_y = map_z = 0.0
        label_3d = ""
        distance = -1.0

        if self.depth_image is not None:
            h, w = self.depth_image.shape
            if 0 <= cx < w and 0 <= cy < h:
                distance = float(self.depth_image[cy, cx])  
                if distance > 0: 
                    fx = 525.0  
                    fy = 525.0
                    cx_optical = w / 2.0
                    cy_optical = h / 2.0

                    z_3d = distance
                    x_3d = (cx - cx_optical) * z_3d / fx
                    y_3d = (cy - cy_optical) * z_3d / fy

                    label_3d = f"X: {x_3d:.2f}, Y: {y_3d:.2f}, Z: {z_3d:.2f} m"
                    cv2.putText(
                        annotated_frame,
                        label_3d,
                        (int(x_min), int(y_max) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 0),
                        2
                    )

                    # Convert in coordinate mondo
                    try:
                        camera_point = geometry_msgs.msg.PointStamped()
                        camera_point.header.stamp = rgb_data.header.stamp
                        camera_point.header.frame_id = "shelfino1/frontal_camera_link_optical"
                        camera_point.point.x = x_3d
                        camera_point.point.y = y_3d
                        camera_point.point.z = z_3d

                        transform = self.tf_buffer.lookup_transform(
                            'map',
                            'shelfino1/frontal_camera_link_optical',  # CORREZIONE
                            rclpy.time.Time())

                        map_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)
                        map_x = map_point.point.x
                        map_y = map_point.point.y
                        map_z = map_point.point.z
                    except Exception as e:
                        self.get_logger().error(f"Transform error: {e}")

        return x_3d, y_3d, z_3d, map_x, map_y, map_z, distance

    def publish_tf(self, x_map: float, y_map: float, z_map: float, object_name: str) -> None:
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"  # Frame di riferimento 
        t.child_frame_id = f"{object_name}_frame"
        
        t.transform.translation.x = x_map
        t.transform.translation.y = y_map
        t.transform.translation.z = z_map
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

    def get_detected_objects(self):
        return self.detected_objects_list

    # == point cloud part ==
    def trigger_pcl_callback(self, request, response):
        self.get_logger().info('PCL Trigger received!')
        self.pcl_triggered = not self.pcl_triggered
        if self.pcl_triggered:
            self.get_logger().info('PCL saving activated.')
            # Salva le point cloud degli oggetti rilevati
            if self.detected_objects_list:
                try:
                    # Istanzia PointCloudManager con il percorso di output
                    self.pcl_manager = PointCloudManager(
                        detected_objects_list=self.detected_objects_list,
                        output_dir=self.ply_out_dir
                    )
                    # Salva i file PLY
                    self.pcl_manager.save_object_pointcloud_to_file()
                    
                    response.success = True
                    response.message = f'PCL saved successfully. Saved {len(self.detected_objects_list)} objects to {self.ply_out_dir}.'
                    self.get_logger().info(response.message)
                except Exception as e:
                    self.get_logger().error(f'Error saving PLY files: {e}')
                    response.success = False
                    response.message = f'Error saving PLY files: {e}'
            else:
                self.get_logger().warn('No detected objects to save.')
                response.success = False
                response.message = 'PCL saving activated but no objects detected.'
        else:
            self.get_logger().info('PCL saving deactivated.')
            response.success = True
            response.message = 'PCL saving deactivated.'
        return response
    
    def trigger_filter_pcl_callback(self, request, response):
        self.get_logger().info('PCL Filter Trigger received!')
        try:
            self.filtering_step()
            response.success = True
            response.message = 'PCL filtering completed successfully.'
        except Exception as e:
            self.get_logger().error(f'Error during PCL filtering: {e}')
            response.success = False
            response.message = f'PCL filtering failed: {e}'
        
        return response
    
    def pointcloud_callback(self, cloud_msg: PointCloud2) -> None:
        self.last_cloud = cloud_msg
    
    def extract_bbox_pointcloud(self, x_min: int, y_min: int, x_max: int, y_max: int) -> list:
        if self.last_cloud is None:
            return []
        
        points_3d = []
        
        x_min = max(0, min(x_min, self.last_cloud.width - 1))
        y_min = max(0, min(y_min, self.last_cloud.height - 1))
        x_max = max(0, min(x_max, self.last_cloud.width - 1))
        y_max = max(0, min(y_max, self.last_cloud.height - 1))
        
        bbox_area = (x_max - x_min) * (y_max - y_min)
        
        try:
            if bbox_area < 1000:
                # bbox piccole
                for y in range(y_min, y_max + 1):
                    for x in range(x_min, x_max + 1):
                        try:
                            point = next(pc2.read_points(
                                self.last_cloud, field_names=("x", "y", "z"),
                                skip_nans=False, uvs=[[x, y]]
                            ), None)
                            
                            if point and len(point) >= 3 and not any(np.isnan(point)):
                                points_3d.append({
                                    'x': float(point[0]), 
                                    'y': float(point[1]),
                                    'z': float(point[2]), 
                                    'pixel_x': x, 
                                    'pixel_y': y
                                })
                        except Exception:
                            continue
            else:
                # Per bbox grandi
                all_points = list(pc2.read_points(
                    self.last_cloud, field_names=("x", "y", "z"), skip_nans=True
                ))
                
                for i, point in enumerate(all_points):
                    if len(point) >= 3:
                        pixel_x = i % self.last_cloud.width
                        pixel_y = i // self.last_cloud.width
                        
                        if x_min <= pixel_x <= x_max and y_min <= pixel_y <= y_max:
                            points_3d.append({
                                'x': float(point[0]), 
                                'y': float(point[1]),
                                'z': float(point[2]), 
                                'pixel_x': pixel_x, 
                                'pixel_y': pixel_y
                            })
        except Exception as e:
            self.get_logger().error(f"Error extracting pointcloud from bbox: {e}")
        
        return points_3d
       
    def filtering_step(self):
        # check if pcl_manager have components 
        if not self.pcl_manager or not hasattr(self.pcl_manager, 'detected_objects_list'):
            self.get_logger().warn('PCL not initialized.')
            raise ValueError('PCL not initialized.')
        
        if not self.pcl_manager.detected_objects_list:
            self.get_logger().warn('No detected objects in PCL Manager to filter.')
            raise ValueError('No detected objects to filter.')
        
        self.get_logger().info(f'Starting filtering for {len(self.pcl_manager.detected_objects_list)} objects.')
        
        for obj in self.pcl_manager.detected_objects_list:
            if 'pcl_object' in obj and obj['pcl_object']:
                
                pcd = obj['pcl_object']
                
                obj['pcl_object'] = self.pcl_manager.clean_and_smooth_point_cloud(pcd)
                self.get_logger().info(f"Filtered object {obj['id']} ({obj['label']})")
        
        
        self.pcl_manager.get_3d_points_from_bbox()
        self.pcl_manager.save_object_pointcloud_to_file(nick_name="filter_pcl")
        # Visualize filtered point clouds
        self.get_logger().info('Visualizing filtered point clouds...')
        # self.pcl_manager.visualize_filtered_objects()
        
        self.get_logger().info('Filtered point clouds saved successfully.')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()