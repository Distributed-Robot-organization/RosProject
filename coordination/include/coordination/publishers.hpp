#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "coordination/server_types.hpp"


#ifndef SERVER_PUBLISHERS_H
#define SERVER_PUBLISHERS_H
void publishMarkers(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                    const std::vector<point_t> points,
                    builtin_interfaces::msg::Time stamp,
                    std_msgs::msg::ColorRGBA color);

void publishVoxelEstimate(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<PointXYZProb>::Ptr normalized_pcl,
                          builtin_interfaces::msg::Time stamp);

void publishVoxelEstimate(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<point_t>::Ptr cloud_voxel,
                          const point_2_norm_cloud_map_t normalized_count_per_voxel,
                          builtin_interfaces::msg::Time stamp);

void publishPoligon(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,Polygon poly, builtin_interfaces::msg::Time stamp, std_msgs::msg::ColorRGBA color);
#endif