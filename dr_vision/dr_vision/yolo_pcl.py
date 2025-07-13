#!/usr/bin/env python3
import os
import cv2
import numpy as np
import struct

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Header

from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import Bool
import tf2_ros

from dr_interfaces.msg import ObjectDetectionBox, ObjectDetectionResult
from message_filters import ApproximateTimeSynchronizer, Subscriber

# Importa le utility per lavorare con PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class YoloPCLNode(Node):
    def __init__(self) -> None:
        super().__init__('yolo_pcl_node')

        self.declare_parameter('rgb_image_topic', 'camera/image_raw')
        self.declare_parameter('point_cloud_topic', 'camera/points')
        self.declare_parameter('info_camera', 'camera/camera_info')
        self.declare_parameter('detection_image_topic', 'dr_vision/yolo_detection_image')
        self.declare_parameter('detection_results_topic', 'dr_vision/yolo_detection_results')

        rgb_topic = self.get_parameter('rgb_image_topic').get_parameter_value().string_value
        pcl_topic = self.get_parameter('point_cloud_topic').get_parameter_value().string_value
        info_camera_topic = self.get_parameter('info_camera').get_parameter_value().string_value
        
        detection_image_topic = self.get_parameter('detection_image_topic').get_parameter_value().string_value
        detection_results_topic = self.get_parameter('detection_results_topic').get_parameter_value().string_value

        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # yolo model
        package_share_directory = get_package_share_directory('dr_vision')
        model_path = os.path.join(package_share_directory, 'models', 'best.pt')
        self.model = YOLO(model_path)
        
        # Subscribers
        self.rgb_subscription = Subscriber(self, Image, rgb_topic, qos_profile=qos_profile)
        self.pcl_subscription = Subscriber(self, PointCloud2, pcl_topic, qos_profile=qos_profile)
        self.info_camera_subscription = self.create_subscription(CameraInfo, info_camera_topic, self.info_camera_callback, qos_profile=qos_profile)
        self.saver_subscription = self.create_subscription(Bool,'saver',self.saver_callback,qos_profile=qos_profile)
        
        # Synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_subscription, self.pcl_subscription],
            queue_size=10,
            slop=0.05,
            allow_headerless=False 
        )
        self.sync.registerCallback(self.sync_callback)
        
        # Publishers
        self.image_publisher = self.create_publisher(Image, detection_image_topic, 1)
        self.detection_publisher = self.create_publisher(ObjectDetectionResult, detection_results_topic, 1)

        # Variables
        self.bridge = CvBridge()
        self.camera_info = None
        self.object_pointclouds = {}
        self.saver = False
        
        self.get_logger().info("YoloPCLNode initialized")

    def sync_callback(self, rgb_msg: Image, pcl_msg: PointCloud2) -> None:
        try: 
            # check if the timestamps of the messages are close enough
            rgb_time = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
            pcl_time = pcl_msg.header.stamp.sec + pcl_msg.header.stamp.nanosec * 1e-9
            time_diff = abs(rgb_time - pcl_time)
            
            self.get_logger().info(f"Synchronized messages - Time difference: {time_diff:.3f}s")
            
            # threshold of 0.1 seconds
            if time_diff > 0.1: 
                self.get_logger().warn(f"Large time difference between synchronized messages: {time_diff:.3f}s")

            self.object_detection(rgb_msg, pcl_msg)

        except Exception as e:
            self.get_logger().error(f"Error in sync_callback: {e}")
    
    
    def object_detection(self, rgb_msg: Image, pcl_msg: PointCloud2):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            results = self.model(cv_image)

            if results and results[0].boxes is not None:
                boxes = results[0].boxes
                annotated_frame = results[0].plot()

                detection_msg = ObjectDetectionResult()
                detection_msg.header = Header()
                detection_msg.header.stamp = rgb_msg.header.stamp
                detection_msg.header.frame_id = "object_detection"

                for i, box in enumerate(boxes):
                    xyxy = box.xyxy[0]
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    label = results[0].names[cls_id] if hasattr(results[0], 'names') else str(cls_id)

                    x_min, y_min, x_max, y_max = xyxy
                    cx = int((x_min + x_max) / 2.0)
                    cy = int((y_min + y_max) / 2.0)

                    cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 0), -1)
                    
                    # extract 3D points from the point cloud within the bounding box
                    bbox_points_3d = self.get_3d_points_from_bbox(pcl_msg, int(x_min), int(y_min), int(x_max), int(y_max))
                    
                    # extract the central point 3D
                    center_point_3d = self.get_3d_point_from_pointcloud(pcl_msg, cx, cy)
                    
                    box_msg = ObjectDetectionBox()
                    box_msg.id = i
                    box_msg.label = label
                    box_msg.confidence = conf
                    box_msg.x_min = float(x_min)
                    box_msg.y_min = float(y_min)
                    box_msg.x_max = float(x_max)
                    box_msg.y_max = float(y_max)
                    
                    
                    if bbox_points_3d:
                        self.get_logger().info(f"Object {label}: Found {len(bbox_points_3d)} 3D points in bounding box")
                        
                        
                        x_coords = [p['x'] for p in bbox_points_3d]
                        y_coords = [p['y'] for p in bbox_points_3d]
                        z_coords = [p['z'] for p in bbox_points_3d]
                        
                        if x_coords and y_coords and z_coords:
                            avg_x = np.mean(x_coords)
                            avg_y = np.mean(y_coords)
                            avg_z = np.mean(z_coords)
                            min_z = np.min(z_coords)
                            max_z = np.max(z_coords)
                            
                            self.get_logger().info(f"  Average position: x={avg_x:.3f}, y={avg_y:.3f}, z={avg_z:.3f}")
                            self.get_logger().info(f"  Depth range: {min_z:.3f} to {max_z:.3f} meters")
                            
                            # how to process the point cloud data save or/and publish in a topic
                            self.process_object_pointcloud(label, bbox_points_3d, i)
                    
                    if center_point_3d is not None:
                        self.get_logger().info(f"  Center point: x={center_point_3d[0]:.3f}, y={center_point_3d[1]:.3f}, z={center_point_3d[2]:.3f}")
                    
                    detection_msg.boxes.append(box_msg)

                self.detection_publisher.publish(detection_msg)
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
                annotated_msg.header = rgb_msg.header  # Mantieni l'header originale
                self.image_publisher.publish(annotated_msg)
            else:
                self.get_logger().info("No objects detected.")

        except Exception as e:
            self.get_logger().error(f"Error in process_yolo_detection: {e}")
    
    
    def get_3d_points_from_bbox(self, pcl_msg: PointCloud2, x_min: int, y_min: int, x_max: int, y_max: int):
        """
        Estrae tutti i punti 3D dalla point cloud che si trovano all'interno della bounding box
        """
        try:
            # check if the point cloud is organized
            x_min = max(0, min(x_min, pcl_msg.width - 1))
            y_min = max(0, min(y_min, pcl_msg.height - 1))
            x_max = max(0, min(x_max, pcl_msg.width - 1))
            y_max = max(0, min(y_max, pcl_msg.height - 1))
            
            # list to store the 3D points
            points_3d = []
            # if the bounding box is small, read points one by one
            if (x_max - x_min) * (y_max - y_min) < 1000:  # Se bbox è piccola
                # generate all pixel coordinates in the bounding box
                pixel_coords = []
                for y in range(y_min, y_max + 1):
                    for x in range(x_min, x_max + 1):
                        pixel_coords.append([x, y])
                
                for x, y in pixel_coords:
                    try:
                        point_index = y * pcl_msg.width + x
                        point_gen = pc2.read_points(pcl_msg, field_names=("x", "y", "z"), skip_nans=False, uvs=[[x, y]])
                        
                        for point in point_gen:
                            if len(point) >= 3 and not (np.isnan(point[0]) or np.isnan(point[1]) or np.isnan(point[2])):
                                points_3d.append({
                                    'x': float(point[0]),
                                    'y': float(point[1]),
                                    'z': float(point[2]),
                                    'pixel_x': x,
                                    'pixel_y': y
                                })
                            break 
                    except Exception as point_error:
                        continue
            
            else:  
                all_points = list(pc2.read_points(pcl_msg, field_names=("x", "y", "z"), skip_nans=True))
                
                for i, point in enumerate(all_points):
                    if len(point) >= 3:
                        pixel_x = i % pcl_msg.width
                        pixel_y = i // pcl_msg.width
                        
                        if x_min <= pixel_x <= x_max and y_min <= pixel_y <= y_max:
                            if not (np.isnan(point[0]) or np.isnan(point[1]) or np.isnan(point[2])):
                                points_3d.append({
                                    'x': float(point[0]),
                                    'y': float(point[1]),
                                    'z': float(point[2]),
                                    'pixel_x': pixel_x,
                                    'pixel_y': pixel_y
                                })
            
            return points_3d
            
        except Exception as e:
            self.get_logger().error(f"Error extracting 3D points from bbox: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            return []
        

    def get_3d_point_from_pointcloud(self, pcl_msg: PointCloud2, x: int, y: int):
        """
        Estrae il punto 3D dalla point cloud alle coordinate pixel (x, y), usato solo per il centro della bounding box
        """
        try:
            # Controlla che le coordinate siano dentro i limiti della point cloud
            if x < 0 or x >= pcl_msg.width or y < 0 or y >= pcl_msg.height:
                return None
            point_index = y * pcl_msg.width + x
            try:
                point_gen = pc2.read_points(pcl_msg, field_names=("x", "y", "z"), skip_nans=False, uvs=[[x, y]])
                
                for point in point_gen:
                    if len(point) >= 3 and not (np.isnan(point[0]) or np.isnan(point[1]) or np.isnan(point[2])):
                        return (float(point[0]), float(point[1]), float(point[2]))
                    break 
                    
            except Exception as read_error:
                try:
                    import struct
                    
                    point_step = pcl_msg.point_step
                    row_step = pcl_msg.row_step
                    offset = y * row_step + x * point_step
                    x_bytes = pcl_msg.data[offset:offset+4]
                    y_bytes = pcl_msg.data[offset+4:offset+8]
                    z_bytes = pcl_msg.data[offset+8:offset+12]
                    
                    if len(x_bytes) == 4 and len(y_bytes) == 4 and len(z_bytes) == 4:
                        x_val = struct.unpack('<f', bytes(x_bytes))[0]
                        y_val = struct.unpack('<f', bytes(y_bytes))[0]
                        z_val = struct.unpack('<f', bytes(z_bytes))[0]
                        
                        if not (np.isnan(x_val) or np.isnan(y_val) or np.isnan(z_val)):
                            return (float(x_val), float(y_val), float(z_val))
                
                except Exception as struct_error:
                    self.get_logger().debug(f"Both read methods failed for point ({x}, {y})")
            
            return None
            
        except Exception as e:
            self.get_logger().error(f"Error extracting 3D point: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            return None


    def info_camera_callback(self, info_data: CameraInfo) -> None:
        """
        Callback per le informazioni della camera
        """
        self.camera_info = info_data
        self.get_logger().info("Camera info received")

    def process_object_pointcloud(self, label: str, points_3d: list, object_id: int):
        try:
            self.object_pointclouds[object_id] = {
                'label': label,
                'points': points_3d,
                'timestamp': self.get_clock().now().to_msg()
            }
            # save PC
            if self.saver:
                self.save_object_pointcloud_to_file(label, points_3d, object_id)
            # publish in a topic
            # self.publish_object_pointcloud(label, points_3d, object_id)
            
        except Exception as e:
            self.get_logger().error(f"Error processing object pointcloud: {e}")
            
    def saver_callback(self, msg: Bool) -> None:
        """
        Callback per il subscriber 'saver'.
        """
        self.saver = msg.data
        self.get_logger().info(f"Save pointcloud set to: {self.saver}")

    def save_object_pointcloud_to_file(self, label: str, points_3d: list, object_id: int):
        """
        Salva la point cloud dell'oggetto su file (formato PLY o PCD)
        """
        try:
            import datetime
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"object_{label}_{object_id}.ply"
            
            with open(filename, 'w') as f:
                # Header PLY
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(points_3d)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                if points_3d and points_3d[0].get('rgb') is not None:
                    f.write("property uchar red\n")
                    f.write("property uchar green\n")
                    f.write("property uchar blue\n")
                f.write("end_header\n")
                
                # Dati dei punti
                for point in points_3d:
                    if point.get('rgb') is not None:
                        # Converti RGB da float a componenti separate
                        rgb = int(point['rgb'])
                        r = (rgb >> 16) & 0xFF
                        g = (rgb >> 8) & 0xFF
                        b = rgb & 0xFF
                        f.write(f"{point['x']:.6f} {point['y']:.6f} {point['z']:.6f} {r} {g} {b}\n")
                    else:
                        f.write(f"{point['x']:.6f} {point['y']:.6f} {point['z']:.6f}\n")
            
            self.get_logger().info(f"Saved {len(points_3d)} points to {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Error saving pointcloud to file: {e}")


    def publish_object_pointcloud(self, label: str, points_3d: list, object_id: int):
        try:
            # Crea un nuovo publisher se non esiste
            topic_name = f"dr_vision/object_{label}_{object_id}_pointcloud"
            
            if not hasattr(self, 'object_pc_publishers'):
                self.object_pc_publishers = {}
            
            if topic_name not in self.object_pc_publishers:
                self.object_pc_publishers[topic_name] = self.create_publisher(PointCloud2, topic_name, 1)
            
            # Crea il messaggio PointCloud2
            pc_msg = PointCloud2()
            pc_msg.header.stamp = self.get_clock().now().to_msg()
            pc_msg.header.frame_id = "camera_link_optical"
            
            # Converti i punti in formato PointCloud2
            pc_msg = pc2.create_cloud_xyz32(pc_msg.header, [(p['x'], p['y'], p['z']) for p in points_3d])
            
            # Pubblica
            self.object_pc_publishers[topic_name].publish(pc_msg)
            
            self.get_logger().info(f"Published {len(points_3d)} points to topic {topic_name}")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing object pointcloud: {e}")

    

def main(args=None):
    rclpy.init(args=args)
    node = YoloPCLNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    