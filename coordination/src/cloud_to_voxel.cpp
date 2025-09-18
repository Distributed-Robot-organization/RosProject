#include "coordination/cloud_to_voxel.hpp"
#include <limits>
CloudToVoxel::CloudToVoxel(float voxel_leaf_size, int maximum_count_per_voxel, float minimum_percentage)
{
    voxel_leaf_size_ = voxel_leaf_size;
    maximum_count_per_voxel_ = maximum_count_per_voxel;
    raw_cloud_ = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>());
    octree_ = pcl::octree::OctreePointCloudSearch<point_t>::Ptr(new pcl::octree::OctreePointCloudSearch<point_t>(voxel_leaf_size_));
    voxel_cloud_ = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>());
    minimum_percentage_ = minimum_percentage;
}

void CloudToVoxel::generateVoxels()
{
    pcl::getMinMax3D(*this->raw_cloud_, p_min_, p_max_);
    centroid_.x = (p_max_.x + p_min_.x) / 2;
    centroid_.y = (p_max_.y + p_min_.y) / 2;
    centroid_.z = (p_max_.z + p_min_.z) / 2;

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

    // std::cout << s_x << " " << s_y << " " << s_z << std::endl;
    // std::cout << e_x << " " << e_y << " " << e_z << std::endl;
    // std::cout << e_x - s_x << " " << e_y - s_y << " " << e_z - s_z << std::endl;

    // std::cout << p_min_ << " " << p_max_ << std::endl;
    // std::cout << it_x << " " << it_y << " " << it_z << std::endl;
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
    long unsigned int minimum_count = 0;
    minimum_count = floor(minimum_percentage_ * running_maximum_count);
    for (auto keyval : count_per_voxel_)
    {
        // normalized_count_per_voxel_.emplace(keyval.first, float(keyval.second) / float(running_maximum_count));
        PointXYZProb p;
        p.x = std::get<0>(keyval.first);
        p.y = std::get<1>(keyval.first);
        p.z = std::get<2>(keyval.first);
        if (keyval.second >= minimum_count)
        {
            p.probability = ((float(keyval.second - minimum_count)) / (float(running_maximum_count - minimum_count))) + minimum_percentage_;
        }
        else
        {
            p.probability = 0.0;
        }
        // std::cout <<"["<< p.probability<<"f "<<keyval.second<< "]";
        probability_pcl_->push_back(p);
    }

    std::cout << "Minimum # of points considered  " << minimum_count << " Maximum # of points counted" << running_maximum_count << std::endl;
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

using namespace Eigen;

int findNearestCluster(const Vector3f &point, const std::vector<Vector3f> &centroids)
{
    float min_dist = std::numeric_limits<float>::max();
    int best_index = -1;
    for (int i = 0; i < centroids.size(); ++i)
    {
        float dist = (point - centroids[i]).squaredNorm();
        if (dist < min_dist)
        {
            min_dist = dist;
            best_index = i;
        }
    }
    return best_index;
}

void kMeansClustering(const std::vector<Vector3f> &points, int K,
                      std::vector<int> &labels, std::vector<Vector3f> &centroids,
                      int max_iterations = 100)
{
    int N = points.size();
    labels.resize(N);

    // Initialize centroids randomly
    centroids.clear();
    for (int i = 0; i < K; ++i)
        centroids.push_back(points[rand() % N]);

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        // Assignment step
        for (int i = 0; i < N; ++i)
        {
            labels[i] = findNearestCluster(points[i], centroids);
        }

        // Update step
        std::vector<Vector3f> new_centroids(K, Vector3f::Zero());
        std::vector<int> counts(K, 0);

        for (int i = 0; i < N; ++i)
        {
            new_centroids[labels[i]] += points[i];
            counts[labels[i]] += 1;
        }

        for (int i = 0; i < K; ++i)
        {
            if (counts[i] > 0)
                centroids[i] = new_centroids[i] / counts[i];
        }
    }
}

std::vector<point_t> CloudToVoxel::getUnderExploredVoxels()
{

    std::vector<Vector3f> points;

    pcl::PointCloud<point_t>::Ptr filtered_voxel_pcl = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>());
    for (auto p : *probability_pcl_)
    {
        point_2_count_map_t::const_iterator pos = count_per_voxel_.find({p.x, p.y, p.z});
        if (pos == count_per_voxel_.end())
        {
            throw std::runtime_error("THIS SHOULD BE IMPOSSIBLE since all the points should be present");
        }
        else
        {
            int value = pos->second;

            if (value>= minimum_count_ && value<=maximum_count_per_voxel_) 
            {
                point_t p_out;
                p_out.x = p.x;
                p_out.y = p.y;
                p_out.z = p.z;
                filtered_voxel_pcl->push_back(p_out);
                points.push_back({p.x, p.y, p.z});
            }
        }
    }

    int K = 2;
    std::vector<int> labels;
    std::vector<Vector3f> centroids;

    srand(time(0));
    kMeansClustering(points, K, labels, centroids);

    //     // Estimate normals
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    // pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // normal_estimator.setSearchMethod(tree);
    // normal_estimator.setInputCloud(filtered_voxel_pcl);
    // normal_estimator.setKSearch(30);
    // normal_estimator.compute(*normals);

    // // Region growing segmentation
    // pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    // reg.setMinClusterSize(50);
    // reg.setMaxClusterSize(10000);
    // reg.setSearchMethod(tree);
    // reg.setNumberOfNeighbours(30);
    // reg.setInputCloud(filtered_voxel_pcl);
    // reg.setInputNormals(normals);
    // reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI); // 3 degrees
    // reg.setCurvatureThreshold(1.0);

    // std::vector<pcl::PointIndices> clusters;
    // reg.extract(clusters);

    // std::cout << "Number of clusters found: " << clusters.size() << std::endl;

    // // Creating the KdTree object for the search method of the extraction
    // pcl::search::KdTree<point_t>::Ptr tree(new pcl::search::KdTree<point_t>);
    // pcl::EuclideanClusterExtraction<point_t> ec;
    // std::vector<pcl::PointIndices> clusters_indices;
    // // There are no point remaining in the point cloud to clusterize

    // tree->setInputCloud(filtered_voxel_pcl);

    // ec.setClusterTolerance(voxel_leaf_size_ * 2); // maximum search distance
    // // ec.setMinClusterSize(min_cluster_size);
    // // ec.setMaxClusterSize(max_cluster_size);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(filtered_voxel_pcl);
    // ec.extract(clusters_indices);
    std::vector<point_t> out_vector;

    for (auto point : centroids)
    {
        point_t p(point[0], point[1], point[2]);
        out_vector.push_back(p);
    }

    // for (auto cluster_idxs : clusters)
    // {
    //     Eigen::Vector4f centroid;
    //     pcl::compute3DCentroid(*filtered_voxel_pcl, cluster_idxs, centroid);
    //     point_t p(centroid[0], centroid[1], centroid[2]);

    //     out_vector.push_back(p);
    // }
    return out_vector;
}

void CloudToVoxel::getBBoxParameters(point_t &min_pt, point_t &max_pt, point_t &centroid)
{
    pcl::getMinMax3D(*this->raw_cloud_, min_pt, max_pt);
    centroid.x = (max_pt.x + min_pt.x) / 2;
    centroid.y = (max_pt.y + min_pt.y) / 2;
    centroid.z = (max_pt.z + min_pt.z) / 2;
}

bool CloudToVoxel::isEstimateSatified() // TODO put a way to visualize the voxels that satisfy the count
{

    bool satisfied = true;
    satisfied_voxels_.reset();
    satisfied_voxels_ = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>());

    for (auto keyval : count_per_voxel_)
    {
        if (keyval.second < maximum_count_per_voxel_)
        {
            satisfied = false;
        }
        else{
            point_t p;
            p.x = std::get<0>(keyval.first);
            p.y = std::get<1>(keyval.first);
            p.z = std::get<2>(keyval.first);
            satisfied_voxels_->push_back(p);
        }
    }
    return satisfied;
}
