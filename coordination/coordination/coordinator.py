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


class CoordinatorPcl(Node):
    def __init__(self):
        super().__init__("coordinator_node")

        # where .ply files are located
        self.ply_directory = "/ros2_ws/src/working_directory/point_cloud/filtered_ply"
        # where to save processed .ply files --> in mesh folder save also the .ply and the mesh files
        self.ply_save_directory = "/ros2_ws/src/working_directory/point_cloud/mesh"
        
        self.next_array_pose = self.create_publisher(PoseArray, "next_array_pose", 10)
        
        self.trigger_coordination_srv = self.create_service(
            Trigger,
            "trigger_coordination_next_pose",
            self.trigger_coordination_next_pose_callback,
        )
        
        self.visualize_raw_ply_srv = self.create_service(
            Trigger,
            "visualize_raw_ply_files",
            self.visualize_raw_ply_callback,
        )

        
        self.ply_files = glob.glob(os.path.join(self.ply_directory, "*.ply"))
        
        self.processor = PointCloudProcessor(
            ply_directory=self.ply_directory,
            ply_save_directory=self.ply_save_directory
        )
        
        self.get_logger().info("CoordinatorPcl node started")

    def trigger_coordination_next_pose_callback(self, request, response):
        self.get_logger().info("Trigger service called, running full pipeline")
        
        try:
            # Run full pipeline
            self.processor.full_pipeline()
            
            cluster_centroids = []
            for cluster in self.processor.clusters:
                cluster_centroids.append(cluster['centroid'])
            
            global_centroid = self.processor.global_centroid
            
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = "map"
            
            # Add cluster centroids
            for centroid in cluster_centroids:
                pose = Pose()
                pose.position.x = float(centroid[0])
                pose.position.y = float(centroid[1])
                pose.position.z = float(centroid[2])
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)
            
            # Add global centroid as last pose
            if global_centroid is not None:
                pose = Pose()
                pose.position.x = float(global_centroid[0])
                pose.position.y = float(global_centroid[1])
                pose.position.z = float(global_centroid[2])
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)
            
            # Publish PoseArray
            self.next_array_pose.publish(pose_array)
            
            self.get_logger().info(
                f"Published {len(cluster_centroids)} cluster centroids + global centroid"
            )
            
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
