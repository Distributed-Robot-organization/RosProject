#include "coordination/cloud_to_voxel.hpp"
#include "coordination/publishers.hpp"
#include "coordination/robot_manager.hpp"

#include <memory>
#include <string>
#include <thread>
#include <vector>

using std::placeholders::_1;

//     Timer
//       ╎
//       ╎
//       ∨
// Consume buffer of point_clouds to update the master point_cloud
//       ╎
//       ╎
//       ∨
// All robot have published at least once ? ╶╶╶NO-╶> Don't do anything yet
//       ╎
//      Yes
//       ╎
//       ∨
// Start voxel estimation

// -------------------- thread-safe queue --------------------
template <typename T>
class ConcurrentQueue // Buffer class to manage multiple subsciptions
{
public:
  void push(T item)
  {
    std::lock_guard<std::mutex> lock(m_);
    q_.push_back(std::move(item));
  }

  bool pop(T &out)
  {
    std::lock_guard<std::mutex> lock(m_);
    if (q_.empty())
      return false;
    out = std::move(q_.front());
    q_.pop_front();
    return true;
  }

private:
  std::mutex m_;
  std::deque<T> q_;
};

class MeshServerNode : public rclcpp::Node
{
public:
  MeshServerNode() : Node("mesh_server_node")
  {
    std::string voxel_topic_out = this->declare_parameter<std::string>("voxel_topic_out", "voxel_estimate_out");
    std::string positions_to_explore_vis = this->declare_parameter<std::string>("positions_to_explore_topic_vis", "positions_to_explore_vis");

    std::string robot_pcl_topic_ = this->declare_parameter<std::string>("pcl_topic_in", "cluster_pcl");
    robot_ids_ = this->declare_parameter<std::vector<std::string>>("robot_ids", std::vector<std::string>{"shelfino1", "pollo"});
    voxel_leaf_size_ = this->declare_parameter<float>("voxel_size", 0.05);
    threshold_count_per_voxel_ = this->declare_parameter<int>("threshold_count_per_voxel", 30);
    minimum_percentage_ = this->declare_parameter<float>("minimum_percentage", 0.1);

    hz_ = this->declare_parameter<int>("hz", 3);
    if (robot_ids_.empty())
    {
      RCLCPP_FATAL(get_logger(), "No robot_ids given!");
      rclcpp::shutdown();
      return;
    }

    pcl_manager_ = new CloudToVoxel(voxel_leaf_size_, threshold_count_per_voxel_, minimum_percentage_);

    for (auto robot : robot_ids_)
    {
      std::string cluster_topic_name = "/" + robot + "/" + robot_pcl_topic_;
      RCLCPP_INFO(this->get_logger(), "Setting up subscriber %s", cluster_topic_name.c_str());
      pcl_sub_vect_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
          cluster_topic_name, 10,
          [this, robot](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
          {
            // The subscribers will push the messages to the node buffer
            buffer_.push(ItemMsg{std::move(msg), robot});
          }));
      robots_pcl_it_.emplace(robot, 0);
    }

    // Optional: publish the voxelised cloud
    voxel_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(voxel_topic_out, 200);
    under_explored_publisher_ = create_publisher<visualization_msgs::msg::Marker>(positions_to_explore_vis, 200);
    points_generic_ = create_publisher<visualization_msgs::msg::Marker>("Generic", 200);

    circle_pub_ = create_publisher<visualization_msgs::msg::Marker>("circle_pub", 200);

    // --- main-loop timer (e.g. 20 Hz) ---
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(std::floor((1 / hz_) * 1000))),
        [this]()
        { server_executor(); });
  }

private:
  struct ItemMsg // Buffer structure
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
    std::string robot_id;
  };
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pcl_sub_vect_;
  float minimum_percentage_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr under_explored_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_generic_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr circle_pub_;

  std::vector<std::string> robot_ids_;
  std::map<std::string, int> robots_pcl_it_;
  bool estimating_;
  float voxel_leaf_size_, hz_;
  int threshold_count_per_voxel_;
  CloudToVoxel *pcl_manager_;
  ConcurrentQueue<ItemMsg> buffer_;
  rclcpp::TimerBase::SharedPtr timer_;

  // #############
  // # MAIN LOOP #
  // #############
  void server_executor()
  {
    ItemMsg item;
    // Check if there is the possibility of starting estimation
    // based on if all the robot sent at least one pointcloud
    bool perhaps_start_estimation = true;
    std_msgs::msg::Header header;

    while (buffer_.pop(item))
    {
      std::string robot_sender = item.robot_id;

      pcl::PointCloud<point_t> received_cloud;
      // 1. Convert ROS ->PCL
      pcl::fromROSMsg(*item.msg, received_cloud);

      robots_pcl_it_[robot_sender] = robots_pcl_it_[robot_sender] + 1;
      header = item.msg->header; // TODO: da rimpiazzare con valori sensati;
      // don't start to estimate until all robot have published at least one pcl

      pcl_manager_->expandPCL(received_cloud);
    }
    for (auto keyval : robots_pcl_it_)
    {

      if (keyval.second == 0)
      {
        RCLCPP_INFO(this->get_logger(), "robot %s didn't sent anything yet, didn't start voxel estimation", keyval.first.c_str());
        perhaps_start_estimation = false;
      }
    }
    if (!estimating_ && perhaps_start_estimation)
    {
      RCLCPP_INFO(this->get_logger(), "received at least one pcl from each robot, starting voxel estimation");
      estimating_ = true;
      pcl_manager_->startEstimating();

      std::ostringstream oss;
      for (auto keyval1 : pcl_manager_->voxel_parameters_)
      {
        oss << keyval1.first << ":\n";
        for (auto keyval2 : keyval1.second)
        {
          oss << keyval2.first << " : " << keyval2.second;
        }
      }
      RCLCPP_INFO(this->get_logger(), "voxel parameters \n%s", oss.str().c_str());
      RCLCPP_INFO(this->get_logger(), "voxel Count %lu", pcl_manager_->voxel_cloud_->size());
      // Compute search radius where the robots should position themselves
      point_t centroid = pcl_manager_->centroid_;
      point_t p_max = pcl_manager_->p_max_;
      point_t p_min = pcl_manager_->p_min_;

      double radius = sqrt(pow(p_max.x - centroid.x, 2) + pow(p_max.y - centroid.y, 2));

      Polygon circle = make_circle(centroid.x, centroid.y, radius, 64);
      std_msgs::msg::ColorRGBA circle_color;
      circle_color.a = 1.0f;
      circle_color.r = .5f;
      circle_color.g = .0f;
      circle_color.b = 0.5f;
      std::vector<point_t> points_to_check;
      points_to_check.push_back(p_max);
            points_to_check.push_back(p_min);
      points_to_check.push_back(centroid);

      publishPoligon(circle_pub_, circle, this->get_clock()->now(), circle_color);
      publishMarkers(points_generic_, points_to_check, this->get_clock()->now(), circle_color);

    }

    if (estimating_)
    {
      // point_2_norm_cloud_map_t *norm = new point_2_norm_cloud_map_t();
      // pcl_manager_->getNormalizedCountPerVoxel(norm);
      pcl_manager_->voxelDensityEstimate();
      pcl::PointCloud<PointXYZProb>::Ptr probability_pcl = pcl_manager_->probability_pcl_;
      pcl::PointCloud<point_t>::Ptr raw_cloud = pcl_manager_->raw_cloud_;
      RCLCPP_INFO(this->get_logger(), "raw Count %lu", raw_cloud->size());
      publishVoxelEstimate(voxel_publisher_, probability_pcl, this->get_clock()->now());
      std::vector<point_t> points_to_check = pcl_manager_->getUnderExploredVoxels();
      std_msgs::msg::ColorRGBA centroid_color;
      centroid_color.a = 1.0f;
      centroid_color.r = .5f;
      centroid_color.g = .5f;
      centroid_color.b = 0.5f;

      publishMarkers(under_explored_publisher_, points_to_check, this->get_clock()->now(), centroid_color);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // Multi-threaded executor so every subscription runs in its own thread
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<MeshServerNode>();
  exec.add_node(node);
  exec.spin();
  // rclcpp::spin(std::make_shared<MeshServerNode>());
  rclcpp::shutdown();
  return 0;
}
