
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include "interfaces_pkg/msg/probability_point.hpp"
#include "interfaces_pkg/msg/probability_pcl.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#define PCL_NO_PRECOMPILE

#ifndef CloudToVoxel_H_Types
#define CloudToVoxel_H_Types
typedef pcl::PointXYZ point_t;
typedef interfaces_pkg::msg::ProbabilityPcl pcl_msg_t;
typedef interfaces_pkg::msg::ProbabilityPoint point_msg_t;
typedef std::map<std::tuple<float, float, float>, int> point_2_count_map_t;
typedef std::map<std::tuple<float, float, float>, float> point_2_norm_cloud_map_t;
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

struct PointXYZProb
{
    PCL_ADD_POINT4D; // preferred way of adding a XYZ+padding
    float probability;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZProb, // here we assume a XYZ + "test" (as fields)
                                  (float, x, x)(float, y, y)(float, z, z)(float, probability, probability))
#endif

void publishPointMarkers(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                    const std::vector<point_t> points,
                    builtin_interfaces::msg::Time stamp,
                    std_msgs::msg::ColorRGBA color);

void publishVoxelEstimate(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<point_t>::Ptr cloud_voxel,
                          const point_2_norm_cloud_map_t normalized_count_per_voxel,
                          const builtin_interfaces::msg::Time stamp);
void publishVoxelEstimate(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<PointXYZProb>::Ptr normalized_pcl,
                          const builtin_interfaces::msg::Time stamp);