#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/common.h>
#include <bits/stdc++.h>
#include <interfaces_pkg/msg/probability_point.hpp>
#include <interfaces_pkg/msg/probability_pcl.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <visualization_msgs/msg/marker.hpp>

typedef pcl::PointXYZ point_t;
typedef interfaces_pkg::msg::ProbabilityPcl pcl_msg_t;
typedef interfaces_pkg::msg::ProbabilityPoint point_msg_t;
typedef std::map<std::tuple<float, float, float>, int> point_2_count_map_t;
typedef std::map<std::tuple<float, float, float>, float> point_2_norm_cloud_map_t;

#define PCL_NO_PRECOMPILE


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
    pcl::PointCloud<PointXYZProb>::Ptr probability_pcl_;
    point_t p_min_, p_max_, centroid_;
    bool do_estimate_ = false;
    long unsigned int running_maximum_count = 0;

    int threshold_count_per_voxel_, minimum_count_;
    std::map<std::string, std::map<std::string, float>> voxel_parameters_;

    void voxelDensityEstimate();
    void generateVoxels();

public:
    void startEstimating();

    CloudToVoxel(float voxel_leaf_size, int maximum_voxel_count, int minumum_count);

    void expandPCL(pcl::PointCloud<point_t> new_cloud);
    point_2_count_map_t getCountPerVoxel();
    point_2_norm_cloud_map_t getNormalizedCountPerVoxel(point_2_norm_cloud_map_t *copy);
    pcl::PointCloud<point_t>::Ptr getVoxelCloud();
   std::vector<point_t> getUnderExploredVoxels();

    friend class MeshServerNode;
};

