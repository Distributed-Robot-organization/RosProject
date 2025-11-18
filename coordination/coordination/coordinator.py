#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import open3d as o3d
import glob
import os
import numpy as np

from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, PoseArray
from coordination.recostruction import PointCloudProcessor
from coordination.build_mesh import BuildMesh
from std_msgs.msg import Bool, Float32

import sys, yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
class CoordinatorPcl(Node):
    def __init__(self):
        super().__init__("coordinator_node")
        # PARAMTERS
        # where .ply files are located
        self.ply_directory = "/ros2_ws/src/working_directory/point_cloud/filtered_ply"
        # where to save processed .ply files --> in mesh folder save also the .ply and the mesh files
        self.ply_save_directory = "/ros2_ws/src/working_directory/mesh"
        self.yaml_file = "/ros2_ws/src/main_logic/config/object_params.yaml"
        self.declare_parameter("robot_namespaces", ["shelfino1"])
        self.robot_namespaces = self.get_parameter("robot_namespaces").value
        self.get_logger().info(f"Robots found: {self.robot_namespaces}")
        
        self.robot_poses = {}
        
        # PUBLISHERS SUBSCRIBERS AND SERVICES
        for ns in self.robot_namespaces:
            self.robot_poses[ns] = None
            self.create_subscription(PoseWithCovarianceStamped,f"/{ns}/amcl_pose",lambda msg, ns=ns: self.pose_callback(msg, ns),
        10
        )
        
        self.next_array_pose = self.create_publisher(PoseArray, "coordination/next_array_pose", 10)
        self.mean_observation_pub = self.create_publisher(Float32, "coordination/mean_observation_object", 10)
        self.tick_service_coordination_pub = self.create_publisher(Bool, "coordination/tick_service_coordination", 10)
        
        self.trigger_coordination_srv = self.create_service(
            Trigger,
            "trigger_coordination_next_pose",
            self.trigger_coordination_next_pose_callback,
        )
        
        self.generate_mesh_srv = self.create_service(
            Trigger,
            "generate_mesh",
            self.generate_mesh_callback,
        )
        
        self.visualize_raw_ply_srv = self.create_service(
            Trigger,
            "visualize_raw_ply_files",
            self.visualize_raw_ply_callback,
        )
        
        self.ply_files = glob.glob(os.path.join(self.ply_directory, "*.ply"))
        
        self.processor = PointCloudProcessor(
            ply_directory=self.ply_directory,
            ply_save_directory=self.ply_save_directory,
            yaml_file=self.yaml_file,
            logger=self.get_logger()
        )
        self.generate_mesher = BuildMesh(ply_save_directory=self.ply_save_directory)
        
        self.get_logger().info("CoordinatorPcl node started")
        
    def pose_callback(self,msg, namespace):
        pose = msg.pose.pose
        self.robot_poses[namespace] = pose
        self.get_logger().info(f"Received pose from {namespace}: {pose.position.x}, {pose.position.y}, {pose.position.z}, {pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w}")
        return pose

    def trigger_coordination_next_pose_callback(self, request, response):
        self.get_logger().info("Trigger service called, running full pipeline")
        
        try:   
            
            # Run full pipeline
            new_points, global_centroid, mean_observation = self.processor.full_pipeline(self.robot_poses)
            
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = "map"
            
            if global_centroid is not None:
                global_pose = Pose()
                global_pose.position.x = float(global_centroid[0])
                global_pose.position.y = float(global_centroid[1])
                global_pose.position.z = float(global_centroid[2])
                global_pose.orientation.w = 1.0
                pose_array.poses.append(global_pose)
                
            for point in new_points:
                pose = Pose()
                pose.position.x = float(point[0])
                pose.position.y = float(point[1])
                pose.position.z = float(point[2])
                pose.orientation.w = 1.0 # Neutral orientation
                pose_array.poses.append(pose)
            
            # Publish mean observation
            if mean_observation is not None:
                mean_obs_msg = Float32()
                mean_obs_msg.data = float(mean_observation)
                self.mean_observation_pub.publish(mean_obs_msg)
                self.get_logger().info(f"Published mean observation: {mean_observation:.4f}")
                
            # Publish PoseArray
            self.next_array_pose.publish(pose_array)
            
            self.get_logger().info(
                f"Published {len(new_points)} cluster centroids + global centroid"
            )
            
            # Publish tick to signal completion
            tick_msg = Bool()
            tick_msg.data = True
            self.tick_service_coordination_pub.publish(tick_msg)
            
            response.success = True
            response.message = f"Pipeline executed. Published {len(pose_array.poses)} poses"
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error in pipeline: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response

    def publish_pose_array(self):
        pose_array = PoseArray()
        self.next_array_pose.publish(pose_array)
        self.get_logger().info("PoseArray published")

    def generate_mesh_callback(self, request, response):
        try:
            self.get_logger().info("Generate mesh service called")
            
            # Generate mesh
            self.generate_mesher.generate_mesh()
            self.generate_mesher.save_mesh()
            self.generate_mesher.visualize_mesh()

            # Publish tick to signal completion
            tick_msg = Bool()
            tick_msg.data = True
            self.tick_service_coordination_pub.publish(tick_msg)
            
            response.success = True
            response.message = "Mesh generated successfully"
            self.get_logger().info(response.message)
            return response
            
        except Exception as e:
            response.success = False
            response.message = f"Error generating mesh: {str(e)}"
            self.get_logger().error(response.message)
            return response

    def visualize_raw_ply_callback(self, request, response):
        try:

            if not self.ply_files:
                response.success = False
                response.message = f"No .ply files found in {self.ply_directory}"
                self.get_logger().warning(response.message)
                return response

            self.get_logger().info(f"Found {len(self.ply_files)} .ply files")

            point_clouds = []
            for ply_file in self.ply_files:
                self.get_logger().info(f"Loading: {ply_file}")
                pcd = o3d.io.read_point_cloud(ply_file)
                point_clouds.append(pcd)
                self.get_logger().info(f"  Points: {len(pcd.points)}")
            o3d.visualization.draw_geometries(
                point_clouds,
                window_name="PLY Files Viewer",
                width=1280,
                height=720,
            )

            # Publish tick to signal completion
            tick_msg = Bool()
            tick_msg.data = True
            self.tick_service_coordination_pub.publish(tick_msg)

            response.success = True
            response.message = f"Visualized {len(self.ply_files)} .ply files"
            return response

        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            return response


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorPcl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
