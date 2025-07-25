#include "coordination/cloud_to_voxel.hpp"

CloudToVoxel::CloudToVoxel(float voxel_leaf_size)
{
    voxel_leaf_size_ = voxel_leaf_size;
    raw_cloud_ = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>());
    octree_ = pcl::octree::OctreePointCloudSearch<point_t>::Ptr(new pcl::octree::OctreePointCloudSearch<point_t>(voxel_leaf_size_));
    voxel_cloud_ = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>());
}

void CloudToVoxel::generateVoxels()
{
    pcl::getMinMax3D(*this->raw_cloud_, p_min_, p_max_);

    // generating centroid of the voxels a bit outside of the cluster to have a bit of margin
    float s_x = p_min_.x - voxel_leaf_size_ / 2;
    float s_y = p_min_.y - voxel_leaf_size_ / 2;
    float s_z = p_min_.z - voxel_leaf_size_ / 2;

    float e_x = (p_max_.x + voxel_leaf_size_ / 2);
    float e_y = (p_max_.y + voxel_leaf_size_ / 2);
    float e_z = (p_max_.z + voxel_leaf_size_ / 2);

    int it_x = std::ceil<int>((e_x - s_x) / voxel_leaf_size_);
    int it_y = std::ceil<int>((e_y - s_y) / voxel_leaf_size_);
    int it_z = std::ceil<int>((e_z - s_z) / voxel_leaf_size_);

    std::cout << s_x << " " << s_y << " " << s_z << std::endl;
    std::cout << e_x << " " << e_y << " " << e_z << std::endl;
    std::cout << e_x - s_x << " " << e_y - s_y << " " << e_z - s_z << std::endl;

    std::cout << p_min_ << " " << p_max_ << std::endl;
    std::cout << it_x << " " << it_y << " " << it_z << std::endl;

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
}

void CloudToVoxel::voxelDensityEstimate()
{
    // search alghorithm that divides the space in cubes with eigth or no children
    // the resolution is the distance between the point and the furthest point it searches
    octree_->setInputCloud(raw_cloud_);
    octree_->addPointsFromInputCloud();
    // Using cannot use directly the point object as key for the map since it cannot
    // understand what is a bigger value (If i understood correctly the error)
    for (auto searchPoint : *voxel_cloud_)
    {
        // Neighbors within voxel search
        std::vector<int> pointIdxVec;
        // It returns the vector of points in voxel_leaf_size_ range from the searchPoint
        if (octree_->voxelSearch(searchPoint, pointIdxVec))
        {
            count_per_voxel_.emplace(std::tuple<float, float, float>(searchPoint.x, searchPoint.y, searchPoint.z), pointIdxVec.size());
            if (maximum_count < pointIdxVec.size())
            {
                maximum_count = pointIdxVec.size();
            }
        }
        else
        {
            count_per_voxel_.emplace(std::tuple<float, float, float>(searchPoint.x, searchPoint.y, searchPoint.z), 0);
        }
    }
    for (auto keyval : count_per_voxel_)
    {
        normalized_count_per_voxel_.emplace(keyval.first, float(keyval.second) / float(maximum_count));
    }
}

void CloudToVoxel::expandPCL(pcl::PointCloud<point_t> new_cloud)
{
    *raw_cloud_ += new_cloud;
    if (do_estimate_)
        this->voxelDensityEstimate();
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

point_2_count_map_t CloudToVoxel::getCountPerVoxel()
{
    point_2_count_map_t *copy(new point_2_count_map_t());
    for (auto keyval : normalized_count_per_voxel_)
    {
        copy->emplace(keyval.first, keyval.second);
    }
    return *copy;
}

pcl::PointCloud<point_t>::Ptr CloudToVoxel::getVoxelCloud()
{

    pcl::PointCloud<point_t>::Ptr copy;
    pcl::copyPointCloud(*voxel_cloud_, *copy);
    return copy;
}

point_2_norm_cloud_map_t CloudToVoxel::getNormalizedCountPerVoxel(point_2_norm_cloud_map_t *copy)
{
    for (auto keyval : normalized_count_per_voxel_)
    {
        copy->emplace(keyval.first, keyval.second);
    }
    return *copy;
}

void publishVoxelEstimate(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<point_t>::Ptr cloud_voxel,
                          const point_2_norm_cloud_map_t normalized_count_per_voxel,
                          builtin_interfaces::msg::Time stamp)
{

    // normalized vector between 0 and 1
    pcl::PointCloud<point_t>::Ptr norm_vox_cloud(new pcl::PointCloud<point_t>((*cloud_voxel).width, (*cloud_voxel).height));
    for (auto p : *cloud_voxel)
    {
        try
        {
            point_t p_out = p;
            p_out.g = uint8_t(255);
            p_out.b = uint8_t(255);
            p_out.r = uint8_t(255);
            p_out.a = static_cast<uint8_t>(std::floor((normalized_count_per_voxel.at(std::tuple<float, float, float>(p.x, p.y, p.z))) * 255));
            norm_vox_cloud->push_back(p_out);
        }
        catch (std::out_of_range &ex)
        {
            std::ostringstream oss;
            oss << "std::out_of_range for reading point " << p.x << " " << p.y << " " << p.z << "Description: " << ex.what();
            throw std::runtime_error(oss.str());
        }
    }

    sensor_msgs::msg::PointCloud2::SharedPtr ros_msg(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*norm_vox_cloud, *ros_msg);
    ros_msg->header.frame_id = "map";
    ros_msg->header.stamp = stamp;
    pub->publish(*ros_msg);
}