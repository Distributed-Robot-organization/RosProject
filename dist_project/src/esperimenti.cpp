float resolution = 128.0f;
// search alghorithm that divides the space in cubes with eigth or no children
// the resolution is the distance between the point and the furthest point it searches
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
octree.setInputCloud(cloud);
octree.addPointsFromInputCloud();
pcl::PointXYZ searchPoint;
searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
// Neighbors within voxel search
std::vector<int>    ;
if (octree.voxelSearch(searchPoint, pointIdxVec))
{
    std::cout << "Neighbors within voxel search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z << ")"
              << std::endl;
    for (std::size_t i = 0; i < pointIdxVec.size(); ++i)
        std::cout << "    " << (*cloud)[pointIdxVec[i]].x
                  << " " << (*cloud)[pointIdxVec[i]].y
                  << " " << (*cloud)[pointIdxVec[i]].z << std::endl;
}
// K nearest neighbor search
int K = 10;
std::vector<int> pointIdxNKNSearch;
std::vector<float> pointNKNSquaredDistance;
std::cout << "K nearest neighbor search at (" << searchPoint.x
          << " " << searchPoint.y
          << " " << searchPoint.z
          << ") with K=" << K << std::endl;
if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
{
    for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
        std::cout << "    " << (*cloud)[pointIdxNKNSearch[i]].x
                  << " " << (*cloud)[pointIdxNKNSearch[i]].y
                  << " " << (*cloud)[pointIdxNKNSearch[i]].z
                  << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
}
// Neighbors within radius search
std::vector<int> pointIdxRadiusSearch;
std::vector<float> pointRadiusSquaredDistance;
float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
std::cout << "Neighbors within radius search at (" << searchPoint.x
          << " " << searchPoint.y
          << " " << searchPoint.z
          << ") with radius=" << radius << std::endl;
if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
{
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        std::cout << "    " << (*cloud)[pointIdxRadiusSearch[i]].x
                  << " " << (*cloud)[pointIdxRadiusSearch[i]].y
                  << " " << (*cloud)[pointIdxRadiusSearch[i]].z
                  << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
}