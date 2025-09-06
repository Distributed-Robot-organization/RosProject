#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/octree/octree_search.h>
#include <pcl/common/common.h>
#include <bits/stdc++.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include "coordination/server_types.hpp"





#ifndef CLOUDTOVOXEL_H
#define CLOUDTOVOXEL_H

class CloudToVoxel
{
private:
    /* data */
    float voxel_leaf_size_;
    float minimum_percentage_;
    pcl::octree::OctreePointCloudSearch<point_t>::Ptr octree_;
    point_2_count_map_t count_per_voxel_;
    point_2_norm_cloud_map_t normalized_count_per_voxel_;
    pcl::PointCloud<point_t>::Ptr raw_cloud_;
    pcl::PointCloud<point_t>::Ptr voxel_cloud_;
    pcl::PointCloud<PointXYZProb>::Ptr probability_pcl_;
    point_t p_min_, p_max_, centroid_;
    bool do_estimate_ = false,densitySatisfied_ = false;
    // long unsigned int running_maximum_count_ = 0;

    int threshold_count_per_voxel_, minimum_count_;
    std::map<std::string, std::map<std::string, float>> voxel_parameters_;

    void voxelDensityEstimate();
    void generateVoxels();

public:
    void startEstimating();

    CloudToVoxel(float voxel_leaf_size, int maximum_voxel_count, float minimum_percentage);
    bool isEstimateSatified();
    void expandPCL(pcl::PointCloud<point_t> new_cloud);
    point_2_count_map_t getCountPerVoxel();
    // point_2_norm_cloud_map_t getNormalizedCountPerVoxel(point_2_norm_cloud_map_t *copy);
    pcl::PointCloud<point_t>::Ptr getVoxelCloud();
    std::vector<point_t> getUnderExploredVoxels();
    void getBBoxParameters(point_t &min_pt, point_t &max_pt, point_t &centroid);

    friend class MeshServerNode;
};
#endif // SERVER_TYPES_H