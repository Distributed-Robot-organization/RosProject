#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/common.h>

#include <interfaces_pkg/msg/probability_point.hpp>
#include <interfaces_pkg/msg/probability_pcl.hpp>

typedef pcl::PointXYZRGBA point_t;
typedef interfaces_pkg::msg::ProbabilityPcl pcl_msg_t;
typedef interfaces_pkg::msg::ProbabilityPoint point_msg_t;

typedef std::map<std::tuple<float, float, float>, int> point_2_count_map_t;
typedef std::map<std::tuple<float, float, float>, float> point_2_norm_cloud_map_t;

#ifndef CloudToVoxel_H
#define CloudToVoxel_H
class CloudToVoxel
{
private:
    /* data */
    float voxel_leaf_size_;
    pcl::octree::OctreePointCloudSearch<point_t>::Ptr octree_;
    point_2_count_map_t count_per_voxel_;
    point_2_norm_cloud_map_t normalized_count_per_voxel_;
    pcl::PointCloud<point_t>::Ptr raw_cloud_;
    pcl::PointCloud<point_t>::Ptr voxel_cloud_;
    point_t p_min_, p_max_, centroid_;
    bool do_estimate_ = false;
    int maximum_count = 0;

    void voxelDensityEstimate();
    void generateVoxels();

public:
    void startEstimating();

    CloudToVoxel(float voxel_leaf_size);

    void expandPCL(pcl::PointCloud<point_t> new_cloud);
    point_2_count_map_t getCountPerVoxel();
    point_2_norm_cloud_map_t getNormalizedCountPerVoxel(point_2_norm_cloud_map_t *copy);
    pcl::PointCloud<point_t>::Ptr getVoxelCloud();

friend class MeshServerNode;
};
void publishVoxelEstimate(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<point_t>::Ptr cloud_voxel,
                          const point_2_norm_cloud_map_t normalized_count_per_voxel,
                          const builtin_interfaces::msg::Time stamp);

#endif