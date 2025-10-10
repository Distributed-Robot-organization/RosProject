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

from cv_bridge import CvBridge
import cv2
import tf2_ros
import tf2_geometry_msgs


from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from vision_system.msg import ObjectDetectionBox, ObjectDetectionResult


class ObjectDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('object_detection_node')
        

        # import the model
        package_share_directory = get_package_share_directory('vision_system')
        model_path = os.path.join(package_share_directory, 'models', 'best.pt')
        self.model = YOLO(model_path)
        
        #for each shelfino
        self.declare_parameter('rgb_image_topic', '/shelfino1/f_camera/image_raw')
        self.declare_parameter('depth_image_topic', '/shelfino1/f_camera/depth/image_raw')
        
        self.declare_parameter('detection_image_topic', 'vision_system/yolo_detection_image')
        self.declare_parameter('detection_results_topic', 'vision_system/yolo_detection_results')
        
        
        # Get parameter values
        rgb_topic = self.get_parameter('rgb_image_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_image_topic').get_parameter_value().string_value
        detection_image_topic = self.get_parameter('detection_image_topic').get_parameter_value().string_value
        detection_results_topic = self.get_parameter('detection_results_topic').get_parameter_value().string_value
        
        
        # QoS settings for the subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=20
        )
        # == SUBSCRIBERS ==
        #subscribe to RGB and depth images using parameters
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
        
        # == PUBLISHER ==
        self.image_publisher = self.create_publisher(Image, detection_image_topic, 1)
        self.detection_publisher = self.create_publisher(ObjectDetectionResult,detection_results_topic, 1)
        
        # == TRIGGER SERVICES ==
        self.srv = self.create_service(Trigger, 'trigger_detection', self.trigger_callback)
        
        self.detection_triggered = False
        
        # == Parameters ==
        self.confidence_threshold = 0.7
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.bridge = CvBridge()
        self.depth_image = None
        self.get_logger().info("Object Detection Node with Depth and map Transform has been started.")
        

    
    def trigger_callback(self, request, response):
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
    
    def depth_callback(self, depth_data: Image) -> None:
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")

    def rgb_callback(self, rgb_data: Image) -> None:
        # Check if detection is triggered
        if not self.detection_triggered:
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        results = self.model(cv_image)

        if len(results) > 0 and results[0].boxes is not None:
            
            boxes = results[0].boxes
            # Inizia con l'immagine originale
            annotated_frame = cv_image.copy()

            detection_msg = ObjectDetectionResult()
            detection_msg.header = Header()
            detection_msg.header.stamp = self.get_clock().now().to_msg()
            detection_msg.header.frame_id = "object_detection"

            for i, box in enumerate(boxes):
                xyxy = box.xyxy[0]
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                
                # Filtra subito gli oggetti sotto la soglia
                if conf < self.confidence_threshold:
                    continue
                
                label = results[0].names[cls_id] if hasattr(results[0], 'names') else str(cls_id)

                x_min, y_min, x_max, y_max = xyxy
                cx = int((x_min + x_max) / 2.0)
                cy = int((y_min + y_max) / 2.0)

                # Disegna solo gli oggetti sopra la soglia
                color = (0, 255, 0)  # Verde
                cv2.rectangle(annotated_frame, 
                            (int(x_min), int(y_min)), 
                            (int(x_max), int(y_max)), 
                            color, 2)
                
                # Label con confidence
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

                self.publish_tf(map_x, map_y, map_z, f"{label}_{i}")

            # Pubblica SOLO se ci sono oggetti sopra la soglia
            if len(detection_msg.boxes) > 0:
                self.detection_publisher.publish(detection_msg)
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
                self.image_publisher.publish(annotated_msg)
            else:
                self.get_logger().info("No objects detected above confidence threshold.")
        else:
            self.get_logger().info("I don't see any objects.")

    def yolo_detection(self, cx, cy, x_min, y_max, annotated_frame, rgb_data):
        """
        Computes 3D position in the camera and map frame from bounding box center.
        """
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
                        camera_point.header.frame_id = "camera_link_optical"
                        camera_point.point.x = x_3d
                        camera_point.point.y = y_3d
                        camera_point.point.z = z_3d

                        transform = self.tf_buffer.lookup_transform(
                            'map',
                            'camera_link_optical',
                            rclpy.time.Time())

                        map_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)
                        map_x = map_point.point.x
                        map_y = map_point.point.y
                        map_z = map_point.point.z
                    except Exception as e:
                        self.get_logger().error(f"Transform error: {e}")

        return x_3d, y_3d, z_3d, map_x, map_y, map_z, distance

    def publish_tf(self, x_map: float, y_map: float, z_map: float, object_name: str) -> None:
        """
        Publishes a static transform (TF) for a detected object.
        """
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