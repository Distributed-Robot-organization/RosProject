#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import Header
import geometry_msgs.msg

import numpy as np
from cv_bridge import CvBridge
import cv2
import tf2_ros
import tf2_geometry_msgs

from sensor_msgs_py import point_cloud2 as pc2
from builtin_interfaces.msg import Time as BuiltinTime
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from vision_system.msg import ObjectDetectionBox, ObjectDetectionResult
from utility_ply import PointCloudManager


class ObjectDetectionNode(Node):

    def __init__(self) -> None:
        super().__init__('object_detection_node')
        
        # Declare namespace parameter first
        self.declare_parameter('robot_namespace', 'shelfino1')
        robot_ns = self.get_parameter('robot_namespace').value
        
        self.ply_detected_dir = "/ros2_ws/src/vision_system/ply_detected"
        self.ply_filtered_dir = "/ros2_ws/src/vision_system/ply_filtered"
        os.makedirs(self.ply_detected_dir, exist_ok=True)
        os.makedirs(self.ply_filtered_dir, exist_ok=True)

        # Load YOLO model
        package_share_directory = get_package_share_directory('vision_system')
        model_path = os.path.join(package_share_directory, 'models', 'best.pt')
        self.model = YOLO(model_path)
        
        # Declare parameters with namespace
        self.declare_parameter('rgb_image_topic', f'/{robot_ns}/f_camera/image_raw')
        self.declare_parameter('depth_image_topic', f'/{robot_ns}/f_camera/depth/image_raw')
        self.declare_parameter('detection_image_topic', f'/{robot_ns}/vision_system/yolo_detection_image')
        self.declare_parameter('detection_results_topic', f'/{robot_ns}/vision_system/yolo_detection_results')
        self.declare_parameter('point_cloud_topic', f'/{robot_ns}/f_camera/points')
        self.declare_parameter('info_camera', f'/{robot_ns}/f_camera/camera_info')
        self.declare_parameter('z_ground_offset', 0.0)

        # Get parameter values
        rgb_topic = self.get_parameter('rgb_image_topic').value
        depth_topic = self.get_parameter('depth_image_topic').value
        image_detection_topic = self.get_parameter('detection_image_topic').value
        detection_results_topic = self.get_parameter('detection_results_topic').value
        point_cloud_topic = self.get_parameter('point_cloud_topic').value
        camera_info_topic = self.get_parameter('info_camera').value
        self.z_ground_offset = self.get_parameter('z_ground_offset').value
        
        # Store robot namespace and optical frame
        self.robot_namespace = robot_ns
        self.camera_optical_frame = f"{robot_ns}/frontal_camera_link_optical"

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=20
        )
        
        # Subscribers
        self.create_subscription(Image, rgb_topic, self.rgb_callback, qos_profile)
        self.create_subscription(Image, depth_topic, self.depth_callback, qos_profile)
        self.create_subscription(PointCloud2, point_cloud_topic, self.pointcloud_callback, qos_profile)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile)
        
        # Publishers
        self.image_detetection_pub = self.create_publisher(Image, image_detection_topic, 1)
        self.detection_res_pub = self.create_publisher(ObjectDetectionResult, detection_results_topic, 1)
        
        # Services
        self.create_service(Trigger, 'trigger_detection', self.trigger_detection_callback)
        self.create_service(Trigger, 'trigger_pcl', self.trigger_pcl_callback)
        self.create_service(Trigger, 'trigger_filter_pcl', self.trigger_filter_pcl_callback)
        
        # State variables
        self.detection_triggered = False
        self.pcl_triggered = False
        self.confidence_threshold = 0.5
        
        # TF and transformation
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Data storage
        self.bridge = CvBridge()
        self.depth_image = None
        self.last_rgb_msg = None
        self.last_cv_image = None
        self.last_cloud = None
        self.last_cam_info = None
        self.detected_objects_list = []
        self.pcl_manager = PointCloudManager()
        
        self.get_logger().info(f"Object Detection Node initialized for robot: {robot_ns}")
        self.get_logger().info(f"PLY detected directory: {self.ply_detected_dir}")
        self.get_logger().info(f"PLY filtered directory: {self.ply_filtered_dir}")
        
    def trigger_detection_callback(self, request, response):
        self.detection_triggered = not self.detection_triggered
        response.success = self.detection_triggered
        response.message = f"Object detection {'activated' if self.detection_triggered else 'deactivated'}"
        self.get_logger().info(response.message)
        return response

    def camera_info_callback(self, info: CameraInfo) -> None:
        self.last_cam_info = info

    def depth_callback(self, depth_data: Image) -> None:
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")

    def rgb_callback(self, rgb_data: Image) -> None:
        if not self.detection_triggered:
            return
        
        self.last_rgb_msg = rgb_data
        self.detected_objects_list.clear()
        
        cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        self.last_cv_image = cv_image
        results = self.model(cv_image)

        if not (len(results) > 0 and results[0].boxes is not None):
            return
        
        boxes = results[0].boxes
        annotated_frame = cv_image.copy()
        detection_msg = ObjectDetectionResult()
        detection_msg.header = Header()
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = "object_detection"

        for i, box in enumerate(boxes):
            conf = float(box.conf[0])
            if conf < self.confidence_threshold:
                continue
            
            xyxy = box.xyxy[0]
            cls_id = int(box.cls[0])
            label = results[0].names[cls_id] if hasattr(results[0], 'names') else str(cls_id)

            x_min, y_min, x_max, y_max = xyxy
            cx = int((x_min + x_max) / 2.0)
            cy = int((y_min + y_max) / 2.0)

            # Draw bounding box
            cv2.rectangle(annotated_frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)
            cv2.putText(annotated_frame, f"{label} {conf:.2f}", (int(x_min), int(y_min) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 0), -1)
            
            # Get 3D coordinates
            x_3d, y_3d, z_3d, map_x, map_y, map_z, distance = self.yolo_detection(
                cx, cy, x_min, y_max, annotated_frame, rgb_data
            )
            
            # Create detection message
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
            
            # Extract and store point cloud
            points_3d = self.extract_bbox_pointcloud(int(x_min), int(y_min), int(x_max), int(y_max))
            
            detected_objects = {
                'id': i,
                'label': label,
                'confidence': conf,
                'bbox': [float(x_min), float(y_min), float(x_max), float(y_max)],
                'distance': distance,
                'world_coordinates': {'x': map_x, 'y': map_y, 'z': map_z},
                'pcl_object': points_3d
            }
            self.detected_objects_list.append(detected_objects)
            self.publish_tf(map_x, map_y, map_z, f"{label}_{i}")

        # Publish results
        if len(detection_msg.boxes) > 0:
            self.detection_res_pub.publish(detection_msg)
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.image_detetection_pub.publish(annotated_msg)

    def yolo_detection(self, cx, cy, x_min, y_max, annotated_frame, rgb_data):
        """Convert pixel coordinates to 3D world coordinates"""
        x_3d = y_3d = z_3d = map_x = map_y = map_z = 0.0
        distance = -1.0

        if self.depth_image is None:
            return x_3d, y_3d, z_3d, map_x, map_y, map_z, distance
        
        h, w = self.depth_image.shape
        if not (0 <= cx < w and 0 <= cy < h):
            return x_3d, y_3d, z_3d, map_x, map_y, map_z, distance
        
        distance = float(self.depth_image[cy, cx])
        if distance <= 0:
            return x_3d, y_3d, z_3d, map_x, map_y, map_z, distance
        
        # Convert to 3D camera coordinates
        fx = fy = 525.0
        cx_optical = w / 2.0
        cy_optical = h / 2.0

        z_3d = distance
        x_3d = (cx - cx_optical) * z_3d / fx
        y_3d = (cy - cy_optical) * z_3d / fy

        cv2.putText(annotated_frame, f"X: {x_3d:.2f}, Y: {y_3d:.2f}, Z: {z_3d:.2f} m",
                   (int(x_min), int(y_max) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Transform to map frame
        try:
            camera_point = geometry_msgs.msg.PointStamped()
            camera_point.header.stamp = rgb_data.header.stamp
            camera_point.header.frame_id = self.camera_optical_frame
            camera_point.point.x = x_3d
            camera_point.point.y = y_3d
            camera_point.point.z = z_3d

            transform = self.tf_buffer.lookup_transform('map', self.camera_optical_frame,
                                                       rclpy.time.Time())
            map_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)
            map_x = map_point.point.x
            map_y = map_point.point.y
            map_z = map_point.point.z
        except Exception as e:
            self.get_logger().error(f"Transform error: {e}")

        return x_3d, y_3d, z_3d, map_x, map_y, map_z, distance

    def publish_tf(self, x_map: float, y_map: float, z_map: float, object_name: str) -> None:
        """Publish TF for detected object"""
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = f"{self.robot_namespace}/{object_name}_frame"
        t.transform.translation.x = x_map
        t.transform.translation.y = y_map
        t.transform.translation.z = z_map
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def get_detected_objects(self):
        return self.detected_objects_list

    def trigger_pcl_callback(self, request, response):
        """Save point clouds of detected objects"""
        self.pcl_triggered = not self.pcl_triggered
        
        if not self.pcl_triggered:
            response.success = True
            response.message = 'PCL saving deactivated'
            self.get_logger().info(response.message)
            return response
        
        if not self.detected_objects_list:
            response.success = False
            response.message = 'No detected objects to save'
            self.get_logger().warn(response.message)
            return response
        
        try:
            # Transform points to map frame
            for obj in self.detected_objects_list:
                if 'pcl_object' in obj and obj['pcl_object']:
                    obj['pcl_object'] = self.transform_points_to_map(obj['pcl_object'])
            
            # Save PLY files
            self.pcl_manager = PointCloudManager(
                detected_objects_list=self.detected_objects_list,
                output_dir=self.ply_detected_dir,
                robot_namespace=self.robot_namespace
            )
            self.pcl_manager.save_object_pointcloud_to_file()
            
            response.success = True
            response.message = f'Saved {len(self.detected_objects_list)} objects to {self.ply_detected_dir}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Error saving PLY files: {e}'
            self.get_logger().error(response.message)
        
        return response
    
    def trigger_filter_pcl_callback(self, request, response):
        """Filter and clean saved point clouds"""
        try:
            self.filtering_step()
            response.success = True
            response.message = 'PCL filtering completed'
        except Exception as e:
            response.success = False
            response.message = f'PCL filtering failed: {e}'
            self.get_logger().error(response.message)
        
        return response
    
    def pointcloud_callback(self, cloud_msg: PointCloud2) -> None:
        self.last_cloud = cloud_msg
    
    def extract_bbox_pointcloud(self, x_min: int, y_min: int, x_max: int, y_max: int) -> list:
        """Extract 3D points within bounding box from point cloud"""
        if self.last_cloud is None:
            return []
        
        points_3d = []
        
        # Clamp coordinates to valid range
        x_min = max(0, min(x_min, self.last_cloud.width - 1))
        y_min = max(0, min(y_min, self.last_cloud.height - 1))
        x_max = max(0, min(x_max, self.last_cloud.width - 1))
        y_max = max(0, min(y_max, self.last_cloud.height - 1))
        
        bbox_area = (x_max - x_min) * (y_max - y_min)
        cloud_timestamp = self.last_cloud.header.stamp
        cloud_frame_id = self.last_cloud.header.frame_id
        
        try:
            if bbox_area < 1000:
                # Small bbox: iterate pixel by pixel
                for y in range(y_min, y_max + 1):
                    for x in range(x_min, x_max + 1):
                        try:
                            point = next(pc2.read_points(
                                self.last_cloud, field_names=("x", "y", "z"),
                                skip_nans=False, uvs=[[x, y]]
                            ), None)
                            
                            if point and len(point) >= 3 and not any(np.isnan(point)):
                                points_3d.append({
                                    'x': float(point[0]), 'y': float(point[1]), 'z': float(point[2]),
                                    'pixel_x': x, 'pixel_y': y,
                                    'frame_id': cloud_frame_id, 'timestamp': cloud_timestamp
                                })
                        except Exception:
                            continue
            else:
                # Large bbox: load all points and filter
                all_points = list(pc2.read_points(self.last_cloud, field_names=("x", "y", "z"), skip_nans=True))
                
                for i, point in enumerate(all_points):
                    if len(point) >= 3:
                        pixel_x = i % self.last_cloud.width
                        pixel_y = i // self.last_cloud.width
                        
                        if x_min <= pixel_x <= x_max and y_min <= pixel_y <= y_max:
                            points_3d.append({
                                'x': float(point[0]), 'y': float(point[1]), 'z': float(point[2]),
                                'pixel_x': pixel_x, 'pixel_y': pixel_y,
                                'frame_id': cloud_frame_id, 'timestamp': cloud_timestamp
                            })
        except Exception as e:
            self.get_logger().error(f"Error extracting pointcloud: {e}")
        
        return points_3d

    def transform_points_to_map(self, points_3d: list) -> list:
        """Transform points from camera frame to map frame"""
        if not points_3d:
            return []
        
        frame_id = points_3d[0].get('frame_id', self.camera_optical_frame)
        timestamp = points_3d[0].get('timestamp')
        
        # Convert timestamp
        ros_time = rclpy.time.Time(seconds=timestamp.sec, nanoseconds=timestamp.nanosec) \
                   if isinstance(timestamp, BuiltinTime) else rclpy.time.Time()
        
        try:
            transform = self.tf_buffer.lookup_transform('map', frame_id, ros_time,
                                                       timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error(f"Transform error: {e}")
            try:
                # Fallback to current time
                transform = self.tf_buffer.lookup_transform('map', frame_id, rclpy.time.Time())
                self.get_logger().warn("Using current time for transform")
            except Exception as e2:
                self.get_logger().error(f"Fallback transform failed: {e2}")
                return points_3d
        
        transformed_points = []
        
        for point in points_3d:
            camera_point = geometry_msgs.msg.PointStamped()
            camera_point.header.stamp = timestamp if isinstance(timestamp, BuiltinTime) else ros_time.to_msg()
            camera_point.header.frame_id = frame_id
            camera_point.point.x = point['x']
            camera_point.point.y = point['y']
            camera_point.point.z = point['z']
            
            try:
                map_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)
                transformed_points.append({
                    'x': float(map_point.point.x),
                    'y': float(map_point.point.y),
                    'z': float(map_point.point.z) + self.z_ground_offset,
                    'pixel_x': point.get('pixel_x'),
                    'pixel_y': point.get('pixel_y')
                })
            except Exception as e:
                self.get_logger().error(f"Error transforming point: {e}")
                continue
        
        return transformed_points

    def filtering_step(self):
        """Apply filtering and smoothing to point clouds"""
        if not self.pcl_manager or not hasattr(self.pcl_manager, 'detected_objects_list'):
            raise ValueError('PCL not initialized')
        
        if not self.pcl_manager.detected_objects_list:
            raise ValueError('No detected objects to filter')
        
        for obj in self.pcl_manager.detected_objects_list:
            if 'pcl_object' in obj and obj['pcl_object']:
                obj['pcl_object'] = self.pcl_manager.clean_and_smooth_point_cloud(obj['pcl_object'])
        
        self.pcl_manager.get_3d_points_from_bbox()
        
        # Change output directory to filtered and save
        self.pcl_manager.output_dir = self.ply_filtered_dir
        self.pcl_manager.save_object_pointcloud_to_file(nick_name="filtered")

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