#include "coordination/cloud_to_voxel.hpp"
#include <limits>
CloudToVoxel::CloudToVoxel(float voxel_leaf_size, int maximum_count_per_voxel, float minimum_percentage)
{
    voxel_leaf_size_ = voxel_leaf_size;
    threshold_count_per_voxel_ = maximum_count_per_voxel;
    raw_cloud_ = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>());
    octree_ = pcl::octree::OctreePointCloudSearch<point_t>::Ptr(new pcl::octree::OctreePointCloudSearch<point_t>(voxel_leaf_size_));
    voxel_cloud_ = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>());
    minimum_percentage_ = minimum_percentage;
}

void CloudToVoxel::generateVoxels()
{
    pcl::getMinMax3D(*this->raw_cloud_, p_min_, p_max_);
    centroid_.x =(p_max_.x + p_min_.x)/2;
    centroid_.y =(p_max_.y + p_min_.y)/2;
    centroid_.z =(p_max_.z + p_min_.z)/2;

    // generating centroid of the voxels a bit outside of the cluster to have a bit of margin
    // starting point of the voxels
    float s_x = p_min_.x - voxel_leaf_size_ / 2;
    float s_y = p_min_.y - voxel_leaf_size_ / 2;
    float s_z = p_min_.z - voxel_leaf_size_ / 2;

    // end points of the voxels

    float e_x = (p_max_.x + voxel_leaf_size_ / 2);
    float e_y = (p_max_.y + voxel_leaf_size_ / 2);
    float e_z = (p_max_.z + voxel_leaf_size_ / 2);
    // Number of iterations needed
    int it_x = std::ceil<int>((e_x - s_x) / voxel_leaf_size_);
    int it_y = std::ceil<int>((e_y - s_y) / voxel_leaf_size_);
    int it_z = std::ceil<int>((e_z - s_z) / voxel_leaf_size_);

    std::cout << s_x << " " << s_y << " " << s_z << std::endl;
    std::cout << e_x << " " << e_y << " " << e_z << std::endl;
    std::cout << e_x - s_x << " " << e_y - s_y << " " << e_z - s_z << std::endl;

    std::cout << p_min_ << " " << p_max_ << std::endl;
    std::cout << it_x << " " << it_y << " " << it_z << std::endl;
    // populating
    for (int i = 0; i < it_x; i++)
    {
        for (int j = 0; j < it_y; j++)
        {
            for (int z = 0; z < it_z; z++)
            {

                voxel_cloud_->push_back(point_t(s_x + i * voxel_leaf_size_, s_y + j * voxel_leaf_size_, s_z + z * voxel_leaf_size_));
            }
        }
    }
    // Debug info of the voxel set

    voxel_parameters_["start"] = {
        {"x", s_x},
        {"y", s_y},
        {"z", s_z}};

    voxel_parameters_["end"] = {
        {"x", e_x},
        {"y", e_y},
        {"z", e_z}};

    voxel_parameters_["iters"] = {
        {"x", static_cast<float>(it_x)},
        {"y", static_cast<float>(it_y)},
        {"z", static_cast<float>(it_z)}};
}

// Is computed the density estimate of voxels that contain a minimum percentage of the maximum
void CloudToVoxel::voxelDensityEstimate()
{
    // search alghorithm that divides the space in cubes with eigth or no children
    // the resolution is the distance between the point and the furthest point it searches
    // octree_ = pcl::octree::OctreePointCloudSearch<point_t>::Ptr(new pcl::octree::OctreePointCloudSearch<point_t>(voxel_leaf_size_));
    octree_->setInputCloud(raw_cloud_);
    octree_->addPointsFromInputCloud();
    probability_pcl_ = pcl::PointCloud<PointXYZProb>::Ptr(new pcl::PointCloud<PointXYZProb>);
    // Cannot use directly the point object as key for the map since it cannot
    // understand what is a bigger value (If i understood correctly the error)
    long unsigned int running_maximum_count = 0;

    for (auto searchPoint : *voxel_cloud_)
    {
        // Neighbors within voxel search
        std::vector<int> pointIdxVec;
        // It returns the vector of points in voxel_leaf_size_ range from the searchPoint
        if (octree_->voxelSearch(searchPoint, pointIdxVec))
        {
            count_per_voxel_.emplace(std::tuple<float, float, float>(searchPoint.x, searchPoint.y, searchPoint.z), pointIdxVec.size());
            if (running_maximum_count < pointIdxVec.size())
            {
                running_maximum_count = pointIdxVec.size();
            }
        }
        else
        {
            count_per_voxel_.emplace(std::tuple<float, float, float>(searchPoint.x, searchPoint.y, searchPoint.z), 0);
        }
    }
    // Get lower limit of the voxels to consider
    long unsigned int minimum_count =0;
    minimum_count = floor(minimum_percentage_*running_maximum_count);
    for (auto keyval : count_per_voxel_)
    {
        // normalized_count_per_voxel_.emplace(keyval.first, float(keyval.second) / float(running_maximum_count));
        PointXYZProb p;
        p.x = std::get<0>(keyval.first);
        p.y = std::get<1>(keyval.first);
        p.z = std::get<2>(keyval.first);
        if (keyval.second>=minimum_count)
        {
            p.probability = ((float(keyval.second-minimum_count))/ (float(running_maximum_count-minimum_count)))+minimum_percentage_;
        }
        else {
            p.probability =0.0;
        }
        std::cout <<"["<< p.probability<<"f "<<keyval.second<< "]";
        probability_pcl_->push_back(p);
    }

    std::cout << "Minimum # of points considered  " <<minimum_count << " Maximum # of points counted" << running_maximum_count<< std::endl;
}

void CloudToVoxel::expandPCL(pcl::PointCloud<point_t> new_cloud)
{
    *raw_cloud_ += new_cloud;
}
void CloudToVoxel::startEstimating()
{
    if (!do_estimate_)
    {
        generateVoxels();
        voxelDensityEstimate();
    }
    do_estimate_ = true;
}

pcl::PointCloud<point_t>::Ptr CloudToVoxel::getVoxelCloud()
{

    pcl::PointCloud<point_t>::Ptr copy;
    pcl::copyPointCloud(*voxel_cloud_, *copy);
    return copy;
}


std::vector<point_t> CloudToVoxel::getUnderExploredVoxels()
{

    pcl::PointCloud<point_t>::Ptr filtered_voxel_pcl = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>());
    for (auto p : *probability_pcl_)
    {
        if (p.probability >= minimum_count_)
        {
            point_t p_out;
            p_out.x = p.x;
            p_out.y = p.y;
            p_out.z = p.z;
            filtered_voxel_pcl->push_back(p_out);
        }
    }
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<point_t>::Ptr tree(new pcl::search::KdTree<point_t>);
    pcl::EuclideanClusterExtraction<point_t> ec;
    std::vector<pcl::PointIndices> clusters_indices;
    // There are no point remaining in the point cloud to clusterize

    tree->setInputCloud(filtered_voxel_pcl);

    ec.setClusterTolerance(voxel_leaf_size_ * 2); // maximum search distance
    // ec.setMinClusterSize(min_cluster_size);
    // ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_voxel_pcl);
    ec.extract(clusters_indices);
    std::vector<point_t> out_vector;
    for (auto cluster_idxs : clusters_indices)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*filtered_voxel_pcl, cluster_idxs, centroid);
        point_t p(centroid[0], centroid[1], centroid[2]);

        out_vector.push_back(p);
    }
    return out_vector;
}
