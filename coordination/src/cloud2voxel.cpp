#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>

#include <interfaces_pkg/msg/probability_point.hpp>
#include <interfaces_pkg/msg/probability_pcl.hpp>


typedef pcl::PointXYZRGBA pcl_type;
typedef interfaces_pkg::msg::ProbabilityPcl pcl_msg_t;
typedef interfaces_pkg::msg::ProbabilityPoint point_msg_t;

using std::placeholders::_1;

class MeshServerNode : public rclcpp::Node
{
public:
    MeshServerNode() : Node("voxel_estimator")
    {
        std::string pcl_topic_in = this->declare_parameter<std::string>("pcl_input", "filtered_pcl");
        std::string voxel_topic_out = this->declare_parameter<std::string>("voxel_topic_out", "voxel_estimate_out");
        voxel_leaf_size_ = this->declare_parameter<float>("voxel_size", 0.05);

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            pcl_topic_in, 10, std::bind(&MeshServerNode::cloudCb, this, _1));

        // Optional: publish the voxelised cloud
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(voxel_topic_out, 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    float voxel_leaf_size_;

    void voxelDensityEstimate(pcl::PointCloud<pcl_type>::Ptr raw_cloud, pcl::PointCloud<pcl_type>::Ptr cloud_voxel, pcl::octree::OctreePointCloudSearch<pcl_type>* octree,
                              std::map<std::tuple<float, float, float>, int>* count_per_voxel, std::map<std::tuple<float, float, float>, float>* normalized_count_per_voxel)
    {
        octree->setInputCloud(raw_cloud);
        octree->addPointsFromInputCloud();
        // Using cannot use directly the point object as key for the map since it cannot 
        // understand what is a bigger value (If i understood correctly the error) 
        int maximum_count = 0;
        for (auto searchPoint : *cloud_voxel)
        {
            // Neighbors within voxel search
            std::vector<int> pointIdxVec;
            // It returns the vector of points in voxel_leaf_size_ range from the searchPoint
            if (octree->voxelSearch(searchPoint, pointIdxVec))
            {
                count_per_voxel->emplace(std::tuple<float, float, float>(searchPoint.x, searchPoint.y,searchPoint.z), pointIdxVec.size());
                if (maximum_count < pointIdxVec.size())
                {
                    maximum_count = pointIdxVec.size();
                }
            }else{
                count_per_voxel->emplace(std::tuple<float, float, float>(searchPoint.x, searchPoint.y, searchPoint.z), 0);
            }
        }
        for (auto keyval : *count_per_voxel)
        {
            normalized_count_per_voxel->emplace(keyval.first, float(keyval.second) / float(maximum_count));
        }
    }
    void publishEstimate(const pcl::PointCloud<pcl_type>::Ptr cloud_voxel, const std::map<std::tuple<float, float, float>, float> *normalized_count_per_voxel, const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {

        // normalized vector between 0 and 1
        pcl::PointCloud<pcl_type>::Ptr norm_vox_cloud(new pcl::PointCloud<pcl_type>((*cloud_voxel).width, (*cloud_voxel).height));
        for (auto p : *cloud_voxel)
        {
            try
            {
                pcl_type p_out = p;
                p_out.g = uint8_t(255);
                p_out.b = uint8_t(255);
                p_out.r = uint8_t(255);
                p_out.a = static_cast<uint8_t>(std::floor((normalized_count_per_voxel->at(std::tuple<float, float, float>(p.x, p.y, p.z))) * 255));
                norm_vox_cloud->push_back(p_out);
            }
            catch (std::out_of_range &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "std::out_of_range with point %f %f %f at line %d.  Description: %s", p.x, p.y, p.z, __LINE__, ex.what());
            }
        }

        sensor_msgs::msg::PointCloud2::SharedPtr ros_msg(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*norm_vox_cloud, *ros_msg);
        ros_msg->header = msg->header;
        ros_msg->header.stamp = this->get_clock()->now();
        pub_->publish(*ros_msg);
    }

    

    void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1. Convert ROS → PCL
        pcl::PointCloud<pcl_type>::Ptr raw_cloud(new pcl::PointCloud<pcl_type>);
        pcl::fromROSMsg(*msg, *raw_cloud);
        //  2. Voxelise
        pcl::VoxelGrid<pcl_type> vg;
        vg.setInputCloud(raw_cloud);
        vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_); // 1 cm leaf size

        pcl::PointCloud<pcl_type>::Ptr cloud_voxel(new pcl::PointCloud<pcl_type>);
        vg.filter(*cloud_voxel); // cloud_voxel now contains one point per voxel (centroid)

        // search alghorithm that divides the space in cubes with eigth or no children
        // the resolution is the distance between the point and the furthest point it searches
        pcl::octree::OctreePointCloudSearch<pcl_type> octree(voxel_leaf_size_);
        std::map<std::tuple<float, float, float>, float> normalized_count_per_voxel;
        std::map<std::tuple<float, float, float>, int> count_per_voxel;
        voxelDensityEstimate(raw_cloud, cloud_voxel, &octree, &count_per_voxel, &normalized_count_per_voxel);
        publishEstimate(cloud_voxel,&normalized_count_per_voxel,msg);

        auto msg_out = pcl_msg_t();
        auto point_msg_out = msg_out.points;

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MeshServerNode>());
    rclcpp::shutdown();
    return 0;
}