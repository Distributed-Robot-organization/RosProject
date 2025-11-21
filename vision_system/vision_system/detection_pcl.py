#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import Header
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import numpy as np
from cv_bridge import CvBridge
import cv2
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs_py import point_cloud2 as pc2
from builtin_interfaces.msg import Time as BuiltinTime
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from vision_system.msg import ObjectDetectionBox, ObjectDetectionResult
from vision_system.srv import NameObject
import torch
import open3d as o3d
import datetime
from std_msgs.msg import Bool

try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False


class ObjectDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('object_detection_node')
        
        # GPU setup
        self.cuda_available = torch.cuda.is_available()
        self.device = 'cuda' if self.cuda_available else 'cpu'
        if self.cuda_available:
            self.get_logger().info(f"CUDA available! Using GPU: {torch.cuda.get_device_name(0)}")
        else:
            self.get_logger().warn("CUDA not available, using CPU")
        
        # Parameters
        self.declare_parameter('robot_namespace', 'shelfino1')
        robot_ns = self.get_parameter('robot_namespace').value
        
        self.declare_parameter('rgb_image_topic', f'/{robot_ns}/f_camera/image_raw')
        self.declare_parameter('depth_image_topic', f'/{robot_ns}/f_camera/depth/image_raw')
        self.declare_parameter('detection_image_topic', f'/{robot_ns}/vision_system/yolo_detection_image')
        self.declare_parameter('detection_results_topic', f'/{robot_ns}/vision_system/yolo_detection_results')
        self.declare_parameter('point_cloud_topic', f'/{robot_ns}/f_camera/points')
        self.declare_parameter('info_camera', f'/{robot_ns}/f_camera/camera_info')
        self.declare_parameter('z_ground_offset', 0.0)
        self.declare_parameter('use_half_precision', True)
        self.declare_parameter('yolo_imgsz', 512)
        self.declare_parameter('voxel_size', 0.01)
        
        rgb_topic = self.get_parameter('rgb_image_topic').value
        depth_topic = self.get_parameter('depth_image_topic').value
        image_detection_topic = self.get_parameter('detection_image_topic').value
        detection_results_topic = self.get_parameter('detection_results_topic').value
        point_cloud_topic = self.get_parameter('point_cloud_topic').value
        camera_info_topic = self.get_parameter('info_camera').value
        odom_topic = f'/{robot_ns}/odom'
        self.z_ground_offset = self.get_parameter('z_ground_offset').value
        self.use_half_precision = self.get_parameter('use_half_precision').value
        self.yolo_imgsz = self.get_parameter('yolo_imgsz').value
        self.voxel_size = self.get_parameter('voxel_size').value
        
        # Output directories
        self.ply_detected_dir = "/ros2_ws/src/working_directory/point_cloud/raw_ply"
        self.ply_filtered_dir = "/ros2_ws/src/working_directory/point_cloud/filtered_ply"
        os.makedirs(self.ply_detected_dir, exist_ok=True)
        os.makedirs(self.ply_filtered_dir, exist_ok=True)

        # YOLO model
        package_share_directory = get_package_share_directory('vision_system')
        model_path = os.path.join(package_share_directory, 'models', 'best.pt')
        self.model = YOLO(model_path)
        if self.cuda_available:
            self.model.to(self.device)
        self.get_logger().info(f"YOLO model loaded on {self.device}")
        
        self.robot_namespace = robot_ns
        self.camera_optical_frame = f"{robot_ns}/frontal_camera_link_optical"

        # QoS profiles
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE,
                                 history=HistoryPolicy.KEEP_LAST, depth=5)
        
        # Subscriptions
        self.create_subscription(Image, rgb_topic, self.rgb_callback, qos_sensor)
        self.create_subscription(Image, depth_topic, self.depth_callback, qos_sensor)
        self.create_subscription(PointCloud2, point_cloud_topic, self.pointcloud_callback, qos_sensor)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, qos_reliable)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, qos_sensor)
        
        # Publishers
        self.image_publisher = self.create_publisher(Image, image_detection_topic, qos_reliable)
        self.detection_publisher = self.create_publisher(ObjectDetectionResult, detection_results_topic, qos_reliable)
        self.tick_service_vision_publisher = self.create_publisher(Bool, f'/{robot_ns}/vision_system/tick_service_vision', 10)

        
        # Services
        self.create_service(NameObject, 'trigger_detection', self.trigger_detection_callback)
        self.create_service(Trigger, 'trigger_pcl', self.trigger_pcl_callback)
        self.create_service(Trigger, 'trigger_filter_pcl', self.trigger_filter_pcl_callback)
        
        # State variables
        self.detection_triggered = False
        self.target_object_name = None  
        self.confidence_threshold = 0.7
        self.detected_objects_list = []
        self.detection_timer = None  
        self.detection_duration = 3.0  # tempo dal primo rilevamento
        
        # TF setup
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Image processing
        self.bridge = CvBridge()
        self.depth_image = None
        self.depth_image_gpu = None
        self.last_rgb_msg = None
        self.last_cloud = None
        
        # Camera intrinsics
        self.fx = self.fy = 525.0
        self.cx_optical = self.cy_optical = None
        
        # Robot position tracking
        self.current_robot_position = None
        
        self.get_logger().info(f"Object Detection Node initialized for robot: {robot_ns}")
        self.get_logger().info("READY TO DETECT FILTER AND SAVE!")

    # === SERVICE CALLBACKS ===
    def trigger_detection_callback(self, request, response):
        object_name = request.name_object.strip()
        
        if not object_name:
            if self.detection_triggered:
                self.get_logger().info('Detection deactivated - extracting 3D points...')
                # Extract 3D points when deactivating
                if self.detected_objects_list and self.last_cloud:
                    for obj in self.detected_objects_list:
                        bbox = obj['bbox']
                        points_3d = self.extract_3d_points_from_bbox(
                            self.last_cloud, int(bbox[0]), int(bbox[1]), 
                            int(bbox[2]), int(bbox[3])
                        )                        
                        # Add metadata for transformation
                        for point in points_3d:
                            point.setdefault('frame_id', self.camera_optical_frame)
                            if 'timestamp' not in point and self.last_rgb_msg:
                                point['timestamp'] = self.last_rgb_msg.header.stamp
                        # Transform to map frame
                        obj['points_3d'] = self.transform_points_to_odom(points_3d)
                        self.get_logger().info(f"Object {obj['label']} (ID {obj['id']}): "
                                             f"Extracted {len(obj['points_3d'])} points")
                
                self.detection_triggered = False
                self.target_object_name = None
                response.success = True
                response.message = 'Detection deactivated'
                
                done_msg = Bool()
                done_msg.data = True
                self.tick_service_vision_publisher.publish(done_msg)
                self.get_logger().info("Detection deactivated, Published tick_service_vision = True")

                
            else:
                response.success = False
                response.message = 'Detection was already inactive. Provide an object name to start detection.'
            
            return response
        
        # Starting detection with specified object name
        self.get_logger().info(f'Detection triggered for object: {object_name}')
        self.detection_triggered = True
        self.target_object_name = object_name.lower()  # Store as lowercase for case-insensitive matching
        self.detected_objects_list.clear()
        
        response.success = True
        response.message = f"Detection activated for object: {object_name}"

        done_msg = Bool()
        done_msg.data = True
        self.tick_service_vision_publisher.publish(done_msg)
        self.get_logger().info("Detection activated, Published tick_service_vision = True")

        return response

    def trigger_pcl_callback(self, request, response):
        self.get_logger().info('PCL Trigger received!')
        if self.detected_objects_list:
            for obj in self.detected_objects_list:
                points_3d = obj.get('points_3d', [])
                if points_3d:
                    self.save_object_pointcloud(self.ply_detected_dir, obj['label'], points_3d, obj['id'])
            response.success = True
            response.message = f'Saved {len(self.detected_objects_list)} objects'
            
            done_msg = Bool()
            done_msg.data = True
            self.tick_service_vision_publisher.publish(done_msg)
            self.get_logger().info("Service raw PCL, Published tick_service_vision = True")
        else:
            response.success = False
            response.message = 'No objects detected'
        return response

    def trigger_filter_pcl_callback(self, request, response):
        self.get_logger().info('Filter PCL Trigger received!')
        try:
            
            if self.detected_objects_list:
                for obj in self.detected_objects_list:
                    points_3d = obj.get('points_3d', [])
                    self.filtering_step(points_3d)
            response.success = True
            response.message = 'PCL filtering completed'
            
            done_msg = Bool()
            done_msg.data = True
            self.tick_service_vision_publisher.publish(done_msg)
            self.get_logger().info(" service pcl filter,Published tick_service_vision = True")

        except Exception as e:
            response.success = False
            response.message = f'PCL filtering failed: {e}'
            self.get_logger().error(response.message)
        
        return response
    
    # === TOPIC CALLBACKS ===
    def odom_callback(self, odom_msg: Odometry) -> None:
        try:
            position = {
                'x': odom_msg.pose.pose.position.x,
                'y': odom_msg.pose.pose.position.y,
                'z': odom_msg.pose.pose.position.z,
                'timestamp': odom_msg.header.stamp,
                'orientation': {
                    'x': odom_msg.pose.pose.orientation.x,
                    'y': odom_msg.pose.pose.orientation.y,
                    'z': odom_msg.pose.pose.orientation.z,
                    'w': odom_msg.pose.pose.orientation.w
                }
            }
            
            self.current_robot_position = position
            
            
        except Exception as e:
            self.get_logger().error(f"Odometry callback error: {e}")

    def camera_info_callback(self, info: CameraInfo) -> None:
        if self.cx_optical is None and len(info.k) >= 5:
            self.fx, self.fy = info.k[0], info.k[4]
            self.cx_optical, self.cy_optical = info.k[2], info.k[5]

    def depth_callback(self, depth_data: Image) -> None:
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, '32FC1')
            if self.cuda_available and self.depth_image_gpu is None:
                if CUPY_AVAILABLE:
                    self.depth_image_gpu = cp.asarray(self.depth_image)
                else:
                    self.depth_image_gpu = torch.from_numpy(self.depth_image).to(self.device)
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def pointcloud_callback(self, cloud_msg: PointCloud2) -> None:
        self.last_cloud = cloud_msg

    def rgb_callback(self, rgb_data: Image) -> None:
        if not self.detection_triggered:
            return
        
        self.last_rgb_msg = rgb_data
        self.detected_objects_list.clear()
        
        cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        
        # YOLO inference
        results = self.model(cv_image, device=self.device, imgsz=self.yolo_imgsz,
                           half=self.use_half_precision and self.cuda_available, verbose=False)

        if not (len(results) > 0 and results[0].boxes is not None):
            return
        
        boxes = results[0].boxes
        annotated_frame = cv_image.copy()
        detection_msg = ObjectDetectionResult()
        detection_msg.header = Header(stamp=self.get_clock().now().to_msg(), 
                                     frame_id="object_detection")
        objects_found = False 

        for i, box in enumerate(boxes):
            conf = float(box.conf[0])
            if conf < self.confidence_threshold:
                continue
                
            xyxy = box.xyxy[0]
            cls_id = int(box.cls[0])
            label = results[0].names[cls_id] if hasattr(results[0], 'names') else str(cls_id)

            # Filter by target object name if specified
            if self.target_object_name and label.lower() != self.target_object_name:
                continue
            
            objects_found = True
            # timer to close after 3s
            if self.detection_triggered and self.detection_timer is None:
                self.get_logger().info("First detection, starting 3s timer to stop detection...")
                self.detection_timer = self.create_timer(self.detection_duration, self.stop_detection_callback)

            x_min, y_min, x_max, y_max = map(float, xyxy)
            cx, cy = int((x_min + x_max) / 2), int((y_min + y_max) / 2)

            # Draw detection
            cv2.rectangle(annotated_frame, (int(x_min), int(y_min)), 
                         (int(x_max), int(y_max)), (0, 255, 0), 2)
            cv2.putText(annotated_frame, f"{label} {conf:.2f}", 
                       (int(x_min), int(y_min) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Get 3D coordinates
            x_3d, y_3d, z_3d, map_x, map_y, map_z, distance = self.get_3d_position(
                cx, cy, x_min, y_max, annotated_frame, rgb_data
            )
            
            # Create message
            box_msg = ObjectDetectionBox()
            box_msg.id = i
            box_msg.label = label
            box_msg.confidence = conf
            box_msg.x_min, box_msg.y_min = x_min, y_min
            box_msg.x_max, box_msg.y_max = x_max, y_max
            box_msg.distance = distance
            box_msg.world_x, box_msg.world_y, box_msg.world_z = map_x, map_y, map_z
            detection_msg.boxes.append(box_msg)
            
            # Store object
            self.detected_objects_list.append({
                'id': i, 'label': label, 'confidence': conf,
                'bbox': [x_min, y_min, x_max, y_max],
                'distance': distance,
                'world_coordinates': {'x': map_x, 'y': map_y, 'z': map_z},
                'points_3d': []
            })
            
            self.publish_tf(map_x, map_y, map_z, f"{label}_{i}")

        if detection_msg.boxes:
            self.detection_publisher.publish(detection_msg)
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.image_publisher.publish(annotated_msg)
            self.get_logger().info(f"Detected {len(detection_msg.boxes)} instances of '{self.target_object_name}'")
       
    def stop_detection_callback(self):
        self.get_logger().info("Stop detection timer end")

        if self.detection_timer is not None:
            self.detection_timer.cancel()
            self.detection_timer = None

        if self.detection_triggered:
            # Create a fake request to trigger the service callback
            from vision_system.srv import NameObject
            fake_request = NameObject.Request()
            fake_request.name_object = "" 
            fake_response = NameObject.Response()
            
            # Call the service callback directly
            self.trigger_detection_callback(fake_request, fake_response)
        
            self.get_logger().info(f"Detection stopped: {fake_response.message}")
              
    def get_3d_position(self, cx, cy, x_min, y_max, frame, rgb_data):
        x_3d = y_3d = z_3d = map_x = map_y = map_z = distance = 0.0

        if self.depth_image is None:
            return x_3d, y_3d, z_3d, map_x, map_y, map_z, distance
        
        h, w = self.depth_image.shape
        if not (0 <= cx < w and 0 <= cy < h):
            return x_3d, y_3d, z_3d, map_x, map_y, map_z, distance
        
        # Get depth
        if self.cuda_available and self.depth_image_gpu is not None:
            distance = float(self.depth_image_gpu[cy, cx].cpu() if not CUPY_AVAILABLE 
                           else self.depth_image_gpu[cy, cx])
        else:
            distance = float(self.depth_image[cy, cx])
            
        if distance <= 0:
            return x_3d, y_3d, z_3d, map_x, map_y, map_z, distance
        
        # Convert to 3D
        cx_opt = self.cx_optical if self.cx_optical else w / 2
        cy_opt = self.cy_optical if self.cy_optical else h / 2
        z_3d = distance
        x_3d = (cx - cx_opt) * z_3d / self.fx
        y_3d = (cy - cy_opt) * z_3d / self.fy

        cv2.putText(frame, f"X:{x_3d:.2f} Y:{y_3d:.2f} Z:{z_3d:.2f}m",
                   (int(x_min), int(y_max) + 20), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.5, (255, 255, 0), 2)

        # Transform to map
        try:
            camera_point = geometry_msgs.msg.PointStamped()
            camera_point.header.stamp = rgb_data.header.stamp
            camera_point.header.frame_id = self.camera_optical_frame
            camera_point.point.x, camera_point.point.y, camera_point.point.z = x_3d, y_3d, z_3d

            transform = self.tf_buffer.lookup_transform('map', self.camera_optical_frame,
                                                       rclpy.time.Time(),
                                                       timeout=rclpy.duration.Duration(seconds=0.1))
            map_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)
            map_x, map_y, map_z = map_point.point.x, map_point.point.y, map_point.point.z
        except Exception as e:
            self.get_logger().debug(f"Transform error: {e}")

        return x_3d, y_3d, z_3d, map_x, map_y, map_z, distance

    def publish_tf(self, x_map: float, y_map: float, z_map: float, name: str) -> None:
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = f"{self.robot_namespace}/{name}_frame"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x_map, y_map, z_map
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def extract_3d_points_from_bbox(self, pcl_msg: PointCloud2, x_min: int, y_min: int, 
                                     x_max: int, y_max: int):
        try:
            # Clamp to bounds
            x_min = max(0, min(x_min, pcl_msg.width - 1))
            y_min = max(0, min(y_min, pcl_msg.height - 1))
            x_max = max(0, min(x_max, pcl_msg.width - 1))
            y_max = max(0, min(y_max, pcl_msg.height - 1))
            
            points_3d = []
            bbox_area = (x_max - x_min) * (y_max - y_min)
            
            if bbox_area < 1000: 
                for y in range(y_min, y_max + 1):
                    for x in range(x_min, x_max + 1):
                        try:
                            for point in pc2.read_points(pcl_msg, field_names=("x", "y", "z"), 
                                                        skip_nans=False, uvs=[[x, y]]):
                                if len(point) >= 3 and not any(np.isnan(point)):
                                    points_3d.append({'x': float(point[0]), 'y': float(point[1]),
                                                    'z': float(point[2]), 'pixel_x': x, 'pixel_y': y})
                                break
                        except:
                            continue
            else:  # Large bbox: read all and filter
                for i, point in enumerate(pc2.read_points(pcl_msg, field_names=("x", "y", "z"), 
                                                         skip_nans=True)):
                    if len(point) >= 3:
                        px, py = i % pcl_msg.width, i // pcl_msg.width
                        if x_min <= px <= x_max and y_min <= py <= y_max:
                            points_3d.append({'x': float(point[0]), 'y': float(point[1]),
                                            'z': float(point[2]), 'pixel_x': px, 'pixel_y': py})
            return points_3d
        except Exception as e:
            self.get_logger().error(f"Point extraction error: {e}")
            return []
        
    def transform_points_to_odom(self, points_3d: list) -> list:
        # now convert points from camera frame to odom frame --> frame odom è uguale a quella di gazebo
        if not points_3d:
            return []

        frame_id = points_3d[0].get("frame_id", self.camera_optical_frame)
        try:
            transform = self.tf_buffer.lookup_transform(
                f"{self.robot_namespace}/odom",  
                frame_id,                        
                rclpy.time.Time(),               
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

        except Exception as e:
            self.get_logger().error(f"[TF] Transform failed: {e}")
            return []

        transformed = []

        for p in points_3d:
            cam_point = geometry_msgs.msg.PointStamped()
            cam_point.header.frame_id = frame_id
            cam_point.header.stamp = self.get_clock().now().to_msg()

            cam_point.point.x = p['x']
            cam_point.point.y = p['y']
            cam_point.point.z = p['z']

            try:
                odom_point = tf2_geometry_msgs.do_transform_point(cam_point, transform)

                transformed.append({
                    'x': float(odom_point.point.x),
                    'y': float(odom_point.point.y),
                    'z': float(odom_point.point.z),
                    'pixel_x': p.get('pixel_x'),
                    'pixel_y': p.get('pixel_y')
                })

            except Exception as e:
                self.get_logger().warn(f"[TF] Point transform failed: {e}")
                continue

        return transformed

    def save_object_pointcloud(self, directory: str, label: str, points_3d: list, obj_id: int):
        try:
            if not points_3d:
                self.get_logger().info(f"VECCHIO NON STO SALVANDO NULLA")
                return
            if directory is None:
                directory = "/ros2_ws/src/working_directory/point_cloud/raw_ply"
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(directory, f"{label}_{obj_id}.ply")
            
            with open(filename, 'w') as f:
                f.write("ply\nformat ascii 1.0\n")
                f.write(f"comment Map frame coordinates\n")
                f.write(f"element vertex {len(points_3d)}\n")
                f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
                for p in points_3d:
                    f.write(f"{p['x']:.6f} {p['y']:.6f} {p['z']:.6f}\n")
            
            self.get_logger().info(f"Saved {len(points_3d)} points to {filename}")
        except Exception as e:
            self.get_logger().error(f"Save error: {e}")

    def filtering_step(self, points_3d: list):
        
        if not self.detected_objects_list:
            self.get_logger().warn("No detected objects to filter")
            return
        
        for obj in self.detected_objects_list:
            points_3d = obj.get('points_3d', [])
            if not points_3d:
                self.get_logger().warn(f"Object {obj['label']} (ID {obj['id']}) has no points to filter")
                continue
            
            self.get_logger().info(f"Starting filtering for {obj['label']} (ID {obj['id']}) with {len(points_3d)} points")
            
            # Convert list to Open3D PointCloud
            points = np.array([[p['x'], p['y'], p['z']] for p in points_3d])
            o3d_pcd = o3d.geometry.PointCloud()
            o3d_pcd.points = o3d.utility.Vector3dVector(points)
            self.get_logger().info(f"Initial point cloud: {len(o3d_pcd.points)} points")
            
            # Step 1: Remove plane background
            o3d_pcd = self.remove_plane_background(o3d_pcd, distance_threshold=0.03)
            self.get_logger().info(f"After plane removal: {len(o3d_pcd.points)} points")
            
            if len(o3d_pcd.points) == 0:
                self.get_logger().warn(f"No points remaining after plane removal for {obj['label']}")
                continue
            
            # Step 1b: Remove floor plane
            o3d_pcd = self.remove_floor_plane(o3d_pcd, floor_z_threshold=0.001, min_height=0.001)
            self.get_logger().info(f"After floor removal: {len(o3d_pcd.points)} points")
            
            if len(o3d_pcd.points) == 0:
                self.get_logger().warn(f"No points remaining after floor removal for {obj['label']}")
                continue
            
            # Step 2: Remove background based on robot position
            o3d_pcd = self.remove_background_from_robot(o3d_pcd, distance_threshold=6.0)
            self.get_logger().info(f"After robot distance filter: {len(o3d_pcd.points)} points")
            
            if len(o3d_pcd.points) == 0:
                self.get_logger().warn(f"No points remaining after robot distance filter for {obj['label']}")
                continue
            
            # Step 3: Remove statistical outliers
            o3d_pcd, _ = o3d_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            self.get_logger().info(f"After outlier removal: {len(o3d_pcd.points)} points")
            
            if len(o3d_pcd.points) == 0:
                self.get_logger().warn(f"No points remaining after outlier removal for {obj['label']}")
                continue
            
            # Step 4: Voxel downsampling
            o3d_pcd = o3d_pcd.voxel_down_sample(voxel_size=self.voxel_size)
            self.get_logger().info(f"After voxel downsampling: {len(o3d_pcd.points)} points")
            
            if len(o3d_pcd.points) == 0:
                self.get_logger().warn(f"No points remaining after downsampling for {obj['label']}")
                continue
            
            # Step 5: Estimate and orient normals
            o3d_pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.voxel_size * 5, max_nn=30
                )
            )
            o3d_pcd.orient_normals_consistent_tangent_plane(30)
            self.get_logger().info(f"Normals estimated and oriented")
            
            # Convert back to list format
            filtered_points = np.asarray(o3d_pcd.points)
            filtered_points_list = [{'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2])} 
                                   for p in filtered_points]
            
            # Update the object with filtered points
            obj['points_3d'] = filtered_points_list
            
            self.get_logger().info(f"Filtering complete: {len(filtered_points_list)} final points")
            
            # Save the filtered point cloud with robot name
            self.save_object_pointcloud(self.ply_filtered_dir, f"{obj['label']}_filtered_{self.robot_namespace}", filtered_points_list, obj['id'])
    
    def remove_plane_background(self, o3d_pcd, distance_threshold=0.1, ransac_n=3, num_iterations=1000):
        # 1. RANSAC to segment the plane
        plane_model, inliers = o3d_pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )
        # set di indici totali
        all_indices = set(range(len(o3d_pcd.points)))
        # Crea un set di punti nel piano
        inlier_indices = set(inliers)
        # Gli outlier sono la differenza: tutti i punti - punti del piano
        outlier_indices = list(all_indices - inlier_indices)
        # Estrai la nuvola di punti utili (quindi toglie il piano)
        pcd_foreground = o3d_pcd.select_by_index(outlier_indices)
        
        if len(pcd_foreground.points) > 0:
            with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):
                labels = np.array(pcd_foreground.cluster_dbscan(eps=0.05, min_points=20))
                
            if len(labels) > 0:
                unique_labels, counts = np.unique(labels, return_counts=True)
                if unique_labels.size > 0 and unique_labels[0] != -1: 
                    largest_cluster_label = unique_labels[np.argmax(counts)]
                    pcd_foreground = pcd_foreground.select_by_index(
                        np.where(labels == largest_cluster_label)[0]
                    )

        return pcd_foreground
    
    
    def remove_floor_plane(self, o3d_pcd, floor_z_threshold=0.001, min_height=0.001):
        if len(o3d_pcd.points) == 0:
            return o3d_pcd
        
        points = np.asarray(o3d_pcd.points)
        
        # Find the minimum Z value (likely the floor)
        min_z = np.min(points[:, 2])
        
        self.get_logger().info(f"Detected floor at Z = {min_z:.3f}m")
        
        # Keep only points above floor + min_height
        mask = points[:, 2] > (min_z + min_height)
        
        # Alternative: use absolute threshold if you know the floor height
        # mask = points[:, 2] > floor_z_threshold
        
        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
        
        # Copy normals if they exist
        if o3d_pcd.has_normals():
            normals = np.asarray(o3d_pcd.normals)
            filtered_pcd.normals = o3d.utility.Vector3dVector(normals[mask])
        
        # Copy colors if they exist
        if o3d_pcd.has_colors():
            colors = np.asarray(o3d_pcd.colors)
            filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
        
        removed_count = len(points) - len(filtered_pcd.points)
        self.get_logger().info(
            f"Removed {removed_count} floor points below Z = {min_z + min_height:.3f}m"
        )
        
        return filtered_pcd
    
    def remove_background_from_robot(self, o3d_pcd, distance_threshold=6.0):
        if self.current_robot_position is None:
            self.get_logger().warn("No robot position available, skipping robot-based background removal")
            return o3d_pcd
        
        if len(o3d_pcd.points) == 0:
            return o3d_pcd
        
        # Get robot position in odom frame
        robot_x = self.current_robot_position['x']
        robot_y = self.current_robot_position['y']
        robot_z = self.current_robot_position['z']
        
        # Convert point cloud to numpy array
        points = np.asarray(o3d_pcd.points)
        
        # Calculate distance from robot to each point (using only x,y for horizontal distance)
        distances = np.sqrt(
            (points[:, 0] - robot_x) ** 2 + 
            (points[:, 1] - robot_y) ** 2
        )
        
        # Filter points within distance threshold
        mask = distances <= distance_threshold
        
        # Create filtered point cloud
        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
        
        # Copy normals if they exist
        if o3d_pcd.has_normals():
            normals = np.asarray(o3d_pcd.normals)
            filtered_pcd.normals = o3d.utility.Vector3dVector(normals[mask])
        
        # Copy colors if they exist
        if o3d_pcd.has_colors():
            colors = np.asarray(o3d_pcd.colors)
            filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
        
        removed_count = len(points) - len(filtered_pcd.points)
        self.get_logger().info(
            f"Removed {removed_count} background points beyond {distance_threshold}m from robot "
            f"(at position [{robot_x:.2f}, {robot_y:.2f}, {robot_z:.2f}])"
        )
        
        return filtered_pcd
    
    
    
    
    def __del__(self):
        if self.cuda_available:
            torch.cuda.empty_cache()


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