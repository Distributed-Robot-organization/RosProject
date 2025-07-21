#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
typedef pcl::PointXYZRGB pcl_type;

using std::placeholders::_1;

class VoxelCounter : public rclcpp::Node
{
public:
    VoxelCounter() : Node("voxel_counter")
    {
        std::string pcl_topic_in = this->declare_parameter<std::string>("pcl_input", "filtered_pcl");
        std::string voxel_topic_out = this->declare_parameter<std::string>("voxel_topic_out", "voxel_estimate_out");
         voxel_leaf_size_ = this->declare_parameter<float>("voxel_size", 0.05);

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            pcl_topic_in, 10, std::bind(&VoxelCounter::cloudCb, this, _1));

        // Optional: publish the voxelised cloud
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(voxel_topic_out, 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    float voxel_leaf_size_;

    void
    cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1. Convert ROS → PCL
        pcl::PointCloud<pcl_type>::Ptr cloud(new pcl::PointCloud<pcl_type>);
        pcl::fromROSMsg(*msg, *cloud);
        //TODO center the centroid of the voxelgrid on the output of the disctributed cluster pcl
        // 2. Voxelise
        pcl::VoxelGrid<pcl_type> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_); // 1 cm leaf size

        pcl::PointCloud<pcl_type>::Ptr cloud_voxel(new pcl::PointCloud<pcl_type>);
        vg.filter(*cloud_voxel); // cloud_voxel now contains one point per voxel (centroid)

        struct VoxelIndex
        {
            int ix, iy, iz;
            bool operator==(const VoxelIndex &other) const
            {
                return ix == other.ix && iy == other.iy && iz == other.iz;
            }
        };

        struct VoxelHash
        {
            std::size_t operator()(const VoxelIndex &v) const
            {
                return ((std::size_t)v.ix * 73856093) ^
                       ((std::size_t)v.iy * 19349663) ^
                       ((std::size_t)v.iz * 83492791);
            }
        };

        std::unordered_map<VoxelIndex, std::size_t, VoxelHash> count;

        float res = 0.01f; // 1 cm
        int max_number_of_points = 0;
        for (const auto &p : *cloud_voxel)
        {
            VoxelIndex v{static_cast<int>(std::floor(p.x / res)),
                         static_cast<int>(std::floor(p.y / res)),
                         static_cast<int>(std::floor(p.z / res))};
            ++count[v];
            if (max_number_of_points < int(count[v]))
                max_number_of_points = count[v];
        }

        std::unordered_map<VoxelIndex, std::float_t, VoxelHash> normalized_count;
        for (auto keyalue : count){
            normalized_count[keyalue.first] = keyalue.second / max_number_of_points;
        }
        // 4. (optional) publish the voxelised cloud
        for (auto p : *cloud_voxel){
            VoxelIndex v{static_cast<int>(std::floor(p.x / res)),
                         static_cast<int>(std::floor(p.y / res)),
                         static_cast<int>(std::floor(p.z / res))};

            p.r = static_cast<int> (std::floor(normalized_count[v] * 256));
            p.g = 0.0;
            p.b = 0.0;
            RCLCPP_INFO(this->get_logger(), "%d", p.r);
        }
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(*cloud_voxel, out);

        out.header = msg->header;
        //TODO set different intensities based on the normalized value of the voxels
        pub_->publish(out);
    }


};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelCounter>());
    rclcpp::shutdown();
    return 0;
}